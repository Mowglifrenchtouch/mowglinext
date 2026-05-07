// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// costmap_scan_filter_node.cpp
//
// Conditional LiDAR preprocessor for the local_costmap and global_costmap
// obstacle layers. Two filters chained on /scan → /scan_costmap:
//
//   1. Dock-blank (stateful)
//      Returns within `dock_blank_range` get +inf'd while the robot is
//      charging or for `post_undock_blank_sec` after charging drops, so
//      the dock body doesn't show up as a LETHAL obstacle to the BackUp
//      recovery's collision checker (which reads local_costmap/costmap_raw).
//      Outside that window, near returns pass through and the existing
//      collision_monitor (which polls /scan unfiltered) handles real-time
//      contact safety.
//
//   2. Ground filter (IMU-aware, slope-tolerant)
//      robot_localization runs in two_d_mode (forces pitch=roll=0 in TF),
//      so on a sloped garden the LiDAR scan plane is physically tilted
//      but TF reports it as horizontal. laser_geometry::projectLaser then
//      projects every return at LIDAR_Z and the obstacle_layer's
//      min_obstacle_height filter does nothing — real ground returns at
//      1–2 m show up as walls and the planner refuses to drive.
//
//      Fix: project each beam ourselves using the latest /imu/data
//      orientation (which carries the actual robot pitch/roll). For each
//      beam we compute the Z of the return relative to the gravity frame:
//        return_Z = lidar_height + range · dz
//        dz       = 2·(qx·qz − qw·qy)·cos α + 2·(qy·qz + qw·qx)·sin α
//      where α is the beam angle and (qw,qx,qy,qz) is the IMU quaternion.
//      Returns whose Z falls outside [min_obstacle_z_m, max_obstacle_z_m]
//      get +inf'd. A 5° pitch at 2 m gives dz ≈ −0.087 → return Z ≈
//      0.22 − 0.17 = 0.05 m, below the 0.08 m floor → filtered. Real
//      obstacles whose top sits well above ground keep returns in-band.
//
//      Falls back to pass-through if no IMU sample within `imu_max_age_s`
//      so we never silently strip obstacles when localization is sick.
//
// collision_monitor still subscribes to /scan unfiltered.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "mowgli_interfaces/msg/status.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

class CostmapScanFilterNode : public rclcpp::Node
{
public:
  CostmapScanFilterNode() : Node("costmap_scan_filter")
  {
    dock_blank_range_ = declare_parameter<double>("dock_blank_range", 0.70);
    post_undock_blank_sec_ = declare_parameter<double>("post_undock_blank_sec", 5.0);
    enable_ground_filter_ = declare_parameter<bool>("enable_ground_filter", true);
    min_obstacle_z_m_ = declare_parameter<double>("min_obstacle_z_m", 0.08);
    max_obstacle_z_m_ = declare_parameter<double>("max_obstacle_z_m", 1.5);
    lidar_height_m_ = declare_parameter<double>("lidar_height_m", 0.22);
    imu_max_age_s_ = declare_parameter<double>("imu_max_age_s", 0.5);
    const std::string input_topic = declare_parameter<std::string>("input_topic", "/scan");
    const std::string output_topic =
        declare_parameter<std::string>("output_topic", "/scan_costmap");
    const std::string status_topic =
        declare_parameter<std::string>("status_topic", "/hardware_bridge/status");
    const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");

    rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(10));
    qos_reliable.reliable();
    qos_reliable.durability_volatile();

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos_sensor);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic,
        qos_sensor,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });

    sub_status_ = create_subscription<mowgli_interfaces::msg::Status>(
        status_topic,
        qos_reliable,
        [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg) { on_status(*msg); });

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        qos_sensor,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(*msg); });

    RCLCPP_INFO(get_logger(),
                "costmap_scan_filter started — %s -> %s, dock_blank_range=%.2f m, "
                "post_undock_blank_sec=%.1f s, ground_filter=%s [Z range %.2f..%.2f m, "
                "lidar_height=%.2f m, imu_max_age=%.2f s, source %s].",
                input_topic.c_str(),
                output_topic.c_str(),
                dock_blank_range_,
                post_undock_blank_sec_,
                enable_ground_filter_ ? "on" : "off",
                min_obstacle_z_m_,
                max_obstacle_z_m_,
                lidar_height_m_,
                imu_max_age_s_,
                imu_topic.c_str());
  }

  // --- Pure logic exposed for unit tests ---------------------------------

  /// Quaternion components in (w, x, y, z) order — matches geometry_msgs.
  struct Quaternion
  {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  /// Ground-filter parameters bundled together so the test can call the
  /// pure filter without a node.
  struct GroundFilterConfig
  {
    bool enabled{false};
    double min_obstacle_z_m{0.08};
    double max_obstacle_z_m{1.5};
    double lidar_height_m{0.22};
  };

  /// Apply the radial blank to a copy of @p in. Returns the result.
  /// `blank_active` is the cached output of `is_blank_active()` — passed
  /// in so the test can drive the state machine without a clock.
  static sensor_msgs::msg::LaserScan filter_scan(const sensor_msgs::msg::LaserScan& in,
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

  /// Apply the IMU-aware ground filter to @p io in place. For each beam,
  /// the projected return Z in a gravity-aligned frame is computed using
  /// the LIDAR scan plane being parallel to base_link XY (flat mount):
  ///
  ///     z_dir = 2·(qx·qz − qw·qy)·cos α + 2·(qy·qz + qw·qx)·sin α
  ///     return_Z = lidar_height + range · z_dir
  ///
  /// where α is the beam angle and (qw,qx,qy,qz) is the IMU quaternion
  /// of base_link in the gravity-aligned (ENU/NED) frame. Returns whose
  /// Z is outside [min_obstacle_z_m, max_obstacle_z_m] are pushed to
  /// +inf so the obstacle_layer ignores them.
  ///
  /// `imu_orientation` is std::nullopt when no fresh IMU sample exists
  /// (or the filter is disabled). In that case the function is a no-op.
  static void apply_ground_filter(sensor_msgs::msg::LaserScan& io,
                                  const GroundFilterConfig& cfg,
                                  const std::optional<Quaternion>& imu_orientation)
  {
    if (!cfg.enabled || !imu_orientation.has_value())
      return;
    const Quaternion q = *imu_orientation;
    // Per-beam Z direction depends only on (cos α, sin α) once the
    // quaternion is fixed; precompute the two coefficients.
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

private:
  void on_status(const mowgli_interfaces::msg::Status& msg)
  {
    const bool is_charging = msg.is_charging;
    if (last_is_charging_known_ && last_is_charging_ && !is_charging)
    {
      // Falling edge — start the post-undock grace window.
      charging_dropped_at_ = now();
      RCLCPP_INFO(get_logger(),
                  "charging dropped — extending dock-blank for %.1f s",
                  post_undock_blank_sec_);
    }
    last_is_charging_ = is_charging;
    last_is_charging_known_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu& msg)
  {
    last_imu_orientation_ =
        Quaternion{msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z};
    last_imu_stamp_ = now();
  }

  void on_scan(const sensor_msgs::msg::LaserScan& msg)
  {
    sensor_msgs::msg::LaserScan out = filter_scan(msg, dock_blank_range_, is_blank_active());

    // Ground filter — only when we have a fresh IMU sample. Stale IMU →
    // pass-through so we never silently strip obstacles when localization
    // is sick (better to have phantom ground returns than to blind the
    // costmap entirely).
    std::optional<Quaternion> imu_q;
    if (enable_ground_filter_ && last_imu_stamp_.nanoseconds() != 0)
    {
      const double age = (now() - last_imu_stamp_).seconds();
      if (age >= 0.0 && age < imu_max_age_s_)
        imu_q = last_imu_orientation_;
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "ground filter idle: last IMU sample %.2fs old (>%.2fs)",
                             age,
                             imu_max_age_s_);
      }
    }
    GroundFilterConfig cfg{enable_ground_filter_,
                           min_obstacle_z_m_,
                           max_obstacle_z_m_,
                           lidar_height_m_};
    apply_ground_filter(out, cfg, imu_q);

    pub_scan_->publish(out);
  }

  bool is_blank_active() const
  {
    if (!last_is_charging_known_)
    {
      // Be conservative until we've heard from hardware_bridge: keep the
      // dock blank in case we're booting docked.
      return true;
    }
    if (last_is_charging_)
      return true;
    if (charging_dropped_at_.nanoseconds() == 0)
      return false;
    const double since_drop = (now() - charging_dropped_at_).seconds();
    return since_drop >= 0.0 && since_drop < post_undock_blank_sec_;
  }

  // --- Subscriptions / publishers ---------------------------------------

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;

  // --- Parameters --------------------------------------------------------

  double dock_blank_range_{0.70};
  double post_undock_blank_sec_{5.0};
  bool enable_ground_filter_{true};
  double min_obstacle_z_m_{0.08};
  double max_obstacle_z_m_{1.5};
  double lidar_height_m_{0.22};
  double imu_max_age_s_{0.5};

  // --- Charging-state machine -------------------------------------------

  bool last_is_charging_{false};
  bool last_is_charging_known_{false};
  rclcpp::Time charging_dropped_at_{0, 0, RCL_ROS_TIME};

  // --- IMU latch (for ground filter) ------------------------------------

  std::optional<Quaternion> last_imu_orientation_;
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::CostmapScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
