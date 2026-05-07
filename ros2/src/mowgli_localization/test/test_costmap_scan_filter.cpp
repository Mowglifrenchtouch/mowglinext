// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// test_costmap_scan_filter.cpp — unit tests for the static filter
// helper in costmap_scan_filter_node. Drives the radial blanking
// directly without instantiating a node, so this stays a pure-C++ test.

#include <cmath>
#include <limits>
#include <optional>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/laser_scan.hpp"

// Expose the static helpers without dragging in rclcpp at link time.
// The implementations live in costmap_scan_filter_node.cpp; we mimic the
// function signatures here and keep them in sync. If the node ever grows
// a third filter pass, refactor these into a shared header instead of
// duplicating again.
namespace mowgli_localization
{
sensor_msgs::msg::LaserScan filter_scan_for_test(const sensor_msgs::msg::LaserScan& in,
                                                 double dock_blank_range,
                                                 bool blank_active)
{
  sensor_msgs::msg::LaserScan out = in;
  if (!blank_active)
    return out;
  const float threshold = static_cast<float>(dock_blank_range);
  const float inf = std::numeric_limits<float>::infinity();
  for (auto& r : out.ranges)
  {
    if (std::isfinite(r) && r < threshold)
      r = inf;
  }
  return out;
}

struct QuaternionForTest
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GroundFilterConfigForTest
{
  bool enabled{false};
  double min_obstacle_z_m{0.08};
  double max_obstacle_z_m{1.5};
  double lidar_height_m{0.22};
};

void apply_ground_filter_for_test(sensor_msgs::msg::LaserScan& io,
                                  const GroundFilterConfigForTest& cfg,
                                  const std::optional<QuaternionForTest>& imu_orientation)
{
  if (!cfg.enabled || !imu_orientation.has_value())
    return;
  const auto& q = *imu_orientation;
  const double k_cos = 2.0 * (q.x * q.z - q.w * q.y);
  const double k_sin = 2.0 * (q.y * q.z + q.w * q.x);
  const float min_z = static_cast<float>(cfg.min_obstacle_z_m);
  const float max_z = static_cast<float>(cfg.max_obstacle_z_m);
  const float inf = std::numeric_limits<float>::infinity();
  const double a0 = io.angle_min;
  const double da = io.angle_increment;
  for (size_t i = 0; i < io.ranges.size(); ++i)
  {
    float& r = io.ranges[i];
    if (!std::isfinite(r))
      continue;
    const double a = a0 + da * static_cast<double>(i);
    const double z_dir = k_cos * std::cos(a) + k_sin * std::sin(a);
    const float return_z = static_cast<float>(cfg.lidar_height_m + r * z_dir);
    if (return_z < min_z || return_z > max_z)
      r = inf;
  }
}

inline QuaternionForTest quat_from_pitch_rad(double pitch_rad)
{
  // Pitch-only quaternion: rotation around base_link Y axis.
  return QuaternionForTest{std::cos(pitch_rad / 2.0), 0.0, std::sin(pitch_rad / 2.0), 0.0};
}
}  // namespace mowgli_localization

namespace
{

sensor_msgs::msg::LaserScan make_scan(const std::vector<float>& ranges)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = -1.57f;
  s.angle_max = 1.57f;
  s.angle_increment = 3.14f / std::max<size_t>(1, ranges.size() - 1);
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges = ranges;
  return s;
}

}  // namespace

TEST(CostmapScanFilter, PassThroughWhenInactive)
{
  auto in = make_scan({0.10f, 0.30f, 0.65f, 1.0f, 5.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, false);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  for (size_t i = 0; i < in.ranges.size(); ++i)
    EXPECT_FLOAT_EQ(out.ranges[i], in.ranges[i]);
}

TEST(CostmapScanFilter, BlanksReturnsBelowThresholdWhenActive)
{
  auto in = make_scan({0.10f, 0.30f, 0.69f, 0.71f, 1.0f, 5.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  EXPECT_FALSE(std::isfinite(out.ranges[0]));
  EXPECT_FALSE(std::isfinite(out.ranges[1]));
  EXPECT_FALSE(std::isfinite(out.ranges[2]));
  EXPECT_FLOAT_EQ(out.ranges[3], 0.71f);
  EXPECT_FLOAT_EQ(out.ranges[4], 1.0f);
  EXPECT_FLOAT_EQ(out.ranges[5], 5.0f);
}

TEST(CostmapScanFilter, LeavesNonFiniteValuesAlone)
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto in = make_scan({inf, nan, 0.20f, 2.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  EXPECT_FALSE(std::isfinite(out.ranges[0]));  // was inf
  EXPECT_TRUE(std::isnan(out.ranges[1]));      // NaN preserved
  EXPECT_FALSE(std::isfinite(out.ranges[2]));  // 0.20 < 0.70 → +inf
  EXPECT_FLOAT_EQ(out.ranges[3], 2.0f);
}

TEST(CostmapScanFilter, ThresholdBoundaryIsExclusive)
{
  // A return exactly equal to the threshold should NOT be blanked
  // (filter uses `r < threshold`, not `<=`).
  auto in = make_scan({0.70f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  EXPECT_FLOAT_EQ(out.ranges[0], 0.70f);
}

// ─────────────────────────────────────────────────────────────────────────
// Ground filter (IMU-aware slope tolerance)
// ─────────────────────────────────────────────────────────────────────────

namespace
{
sensor_msgs::msg::LaserScan make_forward_only_scan(float range_m)
{
  // Single beam pointing along +X (α=0). Easiest to reason about
  // because cos α = 1, sin α = 0 → z_dir = 2·(qx·qz − qw·qy).
  sensor_msgs::msg::LaserScan s;
  s.angle_min = 0.0f;
  s.angle_max = 0.0f;
  s.angle_increment = 0.0f;
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges = {range_m};
  return s;
}
}  // namespace

TEST(CostmapScanFilterGround, NoOpWhenDisabled)
{
  // Even with a steep nose-down pitch, disabled filter must not touch ranges.
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      false, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::QuaternionForTest> q =
      mowgli_localization::quat_from_pitch_rad(0.30);  // ~17° nose-down
  mowgli_localization::apply_ground_filter_for_test(in, cfg, q);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);
}

TEST(CostmapScanFilterGround, NoOpWhenNoImu)
{
  // Filter enabled but no IMU sample → pass-through (failsafe).
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::QuaternionForTest> q;  // empty
  mowgli_localization::apply_ground_filter_for_test(in, cfg, q);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);
}

TEST(CostmapScanFilterGround, FlatGroundReturnPassesThroughOnLevelRobot)
{
  // Identity quaternion (no tilt). Forward beam at 2 m projects to
  // Z = 0.22 + 2·0 = 0.22 m, which sits in [0.08, 1.5] → not filtered.
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::QuaternionForTest> q =
      mowgli_localization::QuaternionForTest{};  // identity
  mowgli_localization::apply_ground_filter_for_test(in, cfg, q);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);
}

TEST(CostmapScanFilterGround, GroundReturnFilteredOnNoseDownSlope)
{
  // Robot pitched nose-down 10°. Forward beam at 2 m: z_dir = -sin(10°) =
  // -0.174 → return Z = 0.22 + 2·(-0.174) = -0.127 m, well below 0.08 m
  // floor → must be filtered to +inf.
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  const double pitch_rad = 10.0 * M_PI / 180.0;  // nose-down (positive in URDF Y rotation)
  std::optional<mowgli_localization::QuaternionForTest> q =
      mowgli_localization::quat_from_pitch_rad(pitch_rad);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, q);
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
}

TEST(CostmapScanFilterGround, NearObstacleSurvivesNoseDownSlope)
{
  // Same 10° nose-down pitch but the return is at 0.5 m. Z = 0.22 +
  // 0.5·(-0.174) = 0.133 m, still above 0.08 floor → keep as obstacle.
  auto in = make_forward_only_scan(0.5f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  const double pitch_rad = 10.0 * M_PI / 180.0;
  std::optional<mowgli_localization::QuaternionForTest> q =
      mowgli_localization::quat_from_pitch_rad(pitch_rad);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, q);
  EXPECT_FLOAT_EQ(in.ranges[0], 0.5f);
}

TEST(CostmapScanFilterGround, OverheadReturnFilteredOnLevelRobot)
{
  // Lift the LIDAR origin by 1.4 m so a 2 m forward beam on a level robot
  // would project to Z = 1.4 m, which is below max_obstacle_z_m (1.5).
  // Push the LIDAR origin to 1.55 m: a 2 m return projects to Z = 1.55 m
  // (level robot), above the 1.5 m ceiling → filtered.
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 1.55};
  std::optional<mowgli_localization::QuaternionForTest> q =
      mowgli_localization::QuaternionForTest{};  // identity (level)
  mowgli_localization::apply_ground_filter_for_test(in, cfg, q);
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
}

TEST(CostmapScanFilterGround, NonFiniteRangesUntouched)
{
  // Inf and NaN beams must remain non-finite regardless of filter logic.
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  sensor_msgs::msg::LaserScan in;
  in.angle_min = 0.0f;
  in.angle_max = static_cast<float>(M_PI);
  in.ranges = {inf, nan, 2.0f};
  in.angle_increment = static_cast<float>(M_PI / 2.0);
  in.range_min = 0.05f;
  in.range_max = 12.0f;
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::QuaternionForTest> q =
      mowgli_localization::quat_from_pitch_rad(0.30);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, q);
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
  EXPECT_TRUE(std::isnan(in.ranges[1]));
}
