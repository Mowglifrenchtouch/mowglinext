// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#ifndef MOWGLI_NAV2_PLUGINS__OBSTACLE_DEVIATION_HPP_
#define MOWGLI_NAV2_PLUGINS__OBSTACLE_DEVIATION_HPP_

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mowgli_nav2_plugins
{

/// Pure-function helpers for the FTC controller's obstacle-deviation
/// behaviour. Kept separate from FTCController so they can be unit-tested
/// against a synthetic Costmap2D without spinning a full controller.
class ObstacleDeviation
{
public:
  /// Lethal-cost threshold (matches nav2_costmap_2d::LETHAL_OBSTACLE).
  static constexpr unsigned char kLethalThreshold = 253u;

  /// Scan path poses [start_idx, start_idx + lookahead_count) and return the
  /// first index whose costmap cell is lethal. Returns -1 if none / costmap
  /// lookup fails.
  static int findFirstObstacleIndex(
      const nav2_costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::msg::PoseStamped>& path,
      std::size_t start_idx,
      int lookahead_count);

  /// Decide which side of `obstacle_pose` is free. Scans perpendicular to
  /// the obstacle's heading by `step` increments out to `max_search`.
  /// Returns the smallest signed offset (positive = left, negative = right)
  /// at which the projected point is in a non-lethal cell. Returns 0.0 if
  /// neither side is reachable within max_search (caller treats as "give up").
  static double chooseDeviationSide(
      const nav2_costmap_2d::Costmap2D& costmap,
      const geometry_msgs::msg::PoseStamped& obstacle_pose,
      double max_search,
      double step);

  /// Check whether the laterally-offset path is clear in the lookahead
  /// window. For each pose in [start_idx, start_idx + lookahead_count), the
  /// pose is shifted perpendicularly by `deviation` (positive = left of
  /// path heading) and the resulting cell is sampled. Returns true if no
  /// sampled cell is lethal.
  static bool isPathClearWithDeviation(
      const nav2_costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::msg::PoseStamped>& path,
      std::size_t start_idx,
      int lookahead_count,
      double deviation);

  /// Search for the smallest |deviation| that makes the path clear, starting
  /// from `initial_deviation` and growing in `step` increments up to
  /// `max_deviation`. Sign is preserved from `initial_deviation` (or chosen
  /// fresh by chooseDeviationSide if 0). Returns the chosen deviation, or
  /// the unchanged max-magnitude value if no clearance found (caller checks
  /// |result| > max_deviation - step).
  static double growDeviationUntilClear(
      const nav2_costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::msg::PoseStamped>& path,
      std::size_t start_idx,
      int lookahead_count,
      double initial_deviation,
      double max_deviation,
      double step);
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__OBSTACLE_DEVIATION_HPP_
