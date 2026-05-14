// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// Implementation of GetRemainingAreaPolygon: returns the area minus the
// already-mowed cells, as a list of disjoint MapArea pieces ready to feed
// to opennav_coverage's ComputeCoveragePath. Boost.Geometry isolated to
// this translation unit to keep the rest of mowgli_map header-clean.

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>

#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/map_types.hpp"

namespace mowgli_map
{

namespace
{
namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
// CCW outer, CW inner — matches both the ROS geometry_msgs/Polygon
// convention used by mowgli_map's area definitions AND F2C's expected
// input (CCW field outer, CW interior cutouts). Boost's default is the
// opposite (CW outer), which silently flipped every Boost.difference
// result polygon and made F2C reject the remaining-area on every
// resume planning call.
using BgPolygon = bg::model::polygon<BgPoint, /*ClockWise=*/false>;
using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;

/// Convert a ROS Polygon into a Boost ring (closed). Empty input → empty ring.
BgPolygon to_bg_polygon(const geometry_msgs::msg::Polygon& poly)
{
  BgPolygon out;
  for (const auto& p : poly.points)
  {
    bg::append(out.outer(), BgPoint(p.x, p.y));
  }
  if (!poly.points.empty())
  {
    const auto& first = poly.points.front();
    const auto& last = poly.points.back();
    if (first.x != last.x || first.y != last.y)
    {
      bg::append(out.outer(), BgPoint(first.x, first.y));
    }
  }
  bg::correct(out);
  return out;
}

/// Convert a Boost ring back to a ROS Polygon. Drops the duplicated closing
/// vertex so consumers can treat the result as an open ring.
geometry_msgs::msg::Polygon to_ros_polygon(const BgPolygon::ring_type& ring)
{
  geometry_msgs::msg::Polygon out;
  size_t n = ring.size();
  if (n >= 2 && ring.front().x() == ring.back().x() &&
      ring.front().y() == ring.back().y())
  {
    --n;
  }
  out.points.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(ring[i].x());
    pt.y = static_cast<float>(ring[i].y());
    pt.z = 0.0f;
    out.points.push_back(pt);
  }
  return out;
}

}  // namespace

void MapServerNode::on_get_remaining_area_polygon(
    const mowgli_interfaces::srv::GetRemainingAreaPolygon::Request::SharedPtr req,
    mowgli_interfaces::srv::GetRemainingAreaPolygon::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_id >= areas_.size())
  {
    res->success = false;
    res->error = "area_id out of range";
    return;
  }

  const auto& area = areas_[req->area_id];
  if (area.is_navigation_area)
  {
    res->success = false;
    res->error = "area is a navigation area, not mowable";
    return;
  }
  if (area.polygon.points.size() < 3)
  {
    res->success = false;
    res->error = "area polygon has fewer than 3 vertices";
    return;
  }
  if (!map_.exists(std::string(layers::MOW_PROGRESS)))
  {
    res->success = false;
    res->error = "mow_progress layer missing";
    return;
  }

  // 1. Build the original polygon as outer ring + original obstacles as holes.
  BgPolygon original = to_bg_polygon(area.polygon);
  for (const auto& obstacle : area.obstacles)
  {
    if (obstacle.points.size() < 3)
    {
      continue;
    }
    BgPolygon obs = to_bg_polygon(obstacle);
    if (obs.outer().empty())
    {
      continue;
    }
    original.inners().emplace_back(obs.outer().begin(), obs.outer().end());
  }
  // Dock body lives on the CLASSIFICATION layer (OBSTACLE_PERMANENT) but
  // remaining_polygon only reads MOW_PROGRESS — so without this hole the
  // F2C planner happily lays swaths over the dock structure. Inject the
  // dock body polygon as an inner ring so the area = mowable_outer minus
  // (operator obstacles ∪ dock body). The corridor is intentionally NOT
  // subtracted: it stays mowable so the robot can drive across it on the
  // way home, just classified as DOCKING_AREA for the keepout carve-out.
  if (has_dock_exclusion_ && dock_body_polygon_.points.size() >= 3)
  {
    BgPolygon dock_body_bg = to_bg_polygon(dock_body_polygon_);
    if (!dock_body_bg.outer().empty())
    {
      original.inners().emplace_back(dock_body_bg.outer().begin(),
                                     dock_body_bg.outer().end());
    }
  }
  bg::correct(original);

  // 2. Walk mow_progress over the area polygon, collect mowed cell positions.
  //    Rule: cell is "mowed" when mow_progress >= 0.3 (matches the rest of
  //    the planner: see is_strip_mowed / compute_coverage_stats).
  const auto& progress = map_[std::string(layers::MOW_PROGRESS)];

  grid_map::Polygon gm_polygon;
  for (const auto& p : area.polygon.points)
  {
    gm_polygon.addVertex(grid_map::Position(p.x, p.y));
  }

  struct CellHit
  {
    int row;
    int col;
    double cx;
    double cy;
  };
  std::vector<CellHit> mowed;
  mowed.reserve(1024);

  for (grid_map::PolygonIterator it(map_, gm_polygon); !it.isPastEnd(); ++it)
  {
    const auto& idx = *it;
    if (progress(idx(0), idx(1)) >= 0.3f)
    {
      grid_map::Position pos;
      if (map_.getPosition(idx, pos))
      {
        mowed.push_back(CellHit{idx(0), idx(1), pos.x(), pos.y()});
      }
    }
  }

  if (mowed.empty())
  {
    // Nothing mowed yet: return the area unchanged as a single piece.
    mowgli_interfaces::msg::MapArea piece;
    piece.name = area.name + "_remaining_0";
    piece.area = area.polygon;
    piece.obstacles = area.obstacles;
    piece.is_navigation_area = false;
    res->pieces.push_back(piece);
    res->success = true;
    return;
  }

  // 3. Compress mowed cells into per-row run-length rectangles, then union
  //    them tree-style (pairwise merge) into a single multipolygon. The
  //    epsilon padding ensures adjacent runs merge cleanly under FP noise.
  std::sort(mowed.begin(), mowed.end(), [](const CellHit& a, const CellHit& b) {
    return std::tie(a.row, a.col) < std::tie(b.row, b.col);
  });

  const double half = 0.5 * resolution_;
  const double eps = 1e-4;  // 0.1 mm overlap to fuse touching rectangles

  std::vector<BgMultiPolygon> rect_mps;
  rect_mps.reserve(mowed.size());

  size_t i = 0;
  while (i < mowed.size())
  {
    size_t j = i + 1;
    while (j < mowed.size() && mowed[j].row == mowed[i].row &&
           mowed[j].col == mowed[j - 1].col + 1)
    {
      ++j;
    }
    const double x_min = std::min(mowed[i].cx, mowed[j - 1].cx) - half - eps;
    const double x_max = std::max(mowed[i].cx, mowed[j - 1].cx) + half + eps;
    const double y_min = std::min(mowed[i].cy, mowed[j - 1].cy) - half - eps;
    const double y_max = std::max(mowed[i].cy, mowed[j - 1].cy) + half + eps;

    BgPolygon rect;
    bg::append(rect.outer(), BgPoint(x_min, y_min));
    bg::append(rect.outer(), BgPoint(x_max, y_min));
    bg::append(rect.outer(), BgPoint(x_max, y_max));
    bg::append(rect.outer(), BgPoint(x_min, y_max));
    bg::append(rect.outer(), BgPoint(x_min, y_min));
    bg::correct(rect);

    BgMultiPolygon mp;
    mp.push_back(std::move(rect));
    rect_mps.push_back(std::move(mp));
    i = j;
  }

  // Pairwise tree merge: O(N log N) unions instead of O(N²) incremental.
  while (rect_mps.size() > 1)
  {
    std::vector<BgMultiPolygon> next;
    next.reserve((rect_mps.size() + 1) / 2);
    for (size_t k = 0; k + 1 < rect_mps.size(); k += 2)
    {
      BgMultiPolygon merged;
      bg::union_(rect_mps[k], rect_mps[k + 1], merged);
      next.push_back(std::move(merged));
    }
    if (rect_mps.size() % 2 == 1)
    {
      next.push_back(std::move(rect_mps.back()));
    }
    rect_mps = std::move(next);
  }
  const BgMultiPolygon& mowed_mp = rect_mps.front();

  // 4. remaining = original - mowed_mp.
  BgMultiPolygon remaining;
  bg::difference(original, mowed_mp, remaining);

  // 5. Convert each output piece to a MapArea. Drop pieces whose area is
  //    smaller than one swath (tool_width²) — they are slivers F2C cannot
  //    cover anyway.
  const double sliver_area_m2 = tool_width_ * tool_width_;
  for (size_t k = 0; k < remaining.size(); ++k)
  {
    const auto& bg_piece = remaining[k];
    if (bg_piece.outer().size() < 4)
    {
      continue;
    }
    const double piece_area = std::abs(bg::area(bg_piece));
    if (piece_area < sliver_area_m2)
    {
      continue;
    }

    mowgli_interfaces::msg::MapArea piece;
    piece.name = area.name + "_remaining_" + std::to_string(res->pieces.size());
    piece.area = to_ros_polygon(bg_piece.outer());
    piece.is_navigation_area = false;
    for (const auto& inner : bg_piece.inners())
    {
      if (inner.size() < 4)
      {
        continue;
      }
      piece.obstacles.push_back(to_ros_polygon(inner));
    }
    if (piece.area.points.size() >= 3)
    {
      res->pieces.push_back(piece);
    }
  }

  res->success = true;
}

}  // namespace mowgli_map
