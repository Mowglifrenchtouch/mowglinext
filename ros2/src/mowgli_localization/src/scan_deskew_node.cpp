// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// scan_deskew_node — undo the rotation smear of a sequential 360° LiDAR
// scan acquired while the robot is rotating.
//
// A rotating LiDAR samples its rays sequentially (one ray every
// `time_increment` seconds). At 10 Hz with 720 rays, the rays spread
// over ~100 ms. While the robot rotates at, say, 0.8 rad/s, the lidar
// orientation drifts ~4.6° during the scan, but every ray is published
// with the SAME header.stamp. Downstream consumers transform the entire
// scan with a single TF lookup, smearing the obstacles by up to ~5° in
// the map frame.
//
// We compensate by rotating each ray's angle in the lidar frame by
// `-ω · dt_i`, where `dt_i = i*time_increment - reference_dt` is the
// ray's time offset relative to a chosen reference (the END of the
// scan here — matches the published header.stamp). The rays are
// re-binned into the original LaserScan grid; each output bin keeps
// the nearest range when multiple input rays remap to it.
//
// Linear motion compensation is currently skipped — at typical
// mowing speeds (≤ 0.3 m/s × ≤ 0.1 s scan) the resulting < 30 mm
// displacement is negligible compared to the rotational smear, and
// adding it requires a per-ray TF lookup or a full pose buffer.
//
// Output topic: `/scan_deskewed` (same structure as input). Configure
// downstream nodes (costmap_scan_filter, collision_monitor sources)
// to read this instead of /scan.

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

class ScanDeskewNode : public rclcpp::Node
{
public:
  ScanDeskewNode() : Node("scan_deskew")
  {
    const std::string input_topic =
        declare_parameter<std::string>("input_topic", "/scan");
    const std::string output_topic =
        declare_parameter<std::string>("output_topic", "/scan_deskewed");
    const std::string imu_topic =
        declare_parameter<std::string>("imu_topic", "/imu/data");

    // Reference time for the deskewed output. "end" keeps the same
    // header.stamp as the input (= timestamp of the last ray); "start"
    // shifts the stamp back by the scan duration so consumers that
    // assume header.stamp = first-ray-time interpret the output
    // correctly. ROS convention is "end", which is what Webots uses.
    reference_ = declare_parameter<std::string>("reference", "end");

    // Maximum age of the cached IMU sample. If the IMU went stale,
    // fall back to passthrough rather than apply a wildly wrong ω.
    imu_max_age_s_ = declare_parameter<double>("imu_max_age_s", 0.5);

    rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos_sensor);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic,
        qos_sensor,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        qos_sensor,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
        {
          latest_omega_z_ = msg->angular_velocity.z;
          latest_imu_t_ = msg->header.stamp;
          have_imu_ = true;
        });

    pub_count_ = 0;
    skipped_count_ = 0;
    create_wall_timer(std::chrono::seconds(15),
                      [this]() {
                        RCLCPP_INFO(get_logger(),
                                    "scan_deskew stats: published=%zu, "
                                    "passthrough(stale-IMU)=%zu, last_ω_z=%.3f rad/s",
                                    pub_count_, skipped_count_, latest_omega_z_);
                      });

    RCLCPP_INFO(get_logger(),
                "scan_deskew started — %s -> %s (reference=%s, imu=%s, max_age=%.2fs)",
                input_topic.c_str(),
                output_topic.c_str(),
                reference_.c_str(),
                imu_topic.c_str(),
                imu_max_age_s_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan& in)
  {
    sensor_msgs::msg::LaserScan out = in;

    // If we have no fresh IMU, pass through unchanged. Better to emit a
    // smeared scan than a wildly mis-corrected one.
    if (!have_imu_)
    {
      pub_->publish(out);
      skipped_count_++;
      return;
    }
    const double imu_age = (now() - latest_imu_t_).seconds();
    if (imu_age < 0.0 || imu_age > imu_max_age_s_)
    {
      pub_->publish(out);
      skipped_count_++;
      return;
    }

    const size_t n = in.ranges.size();
    if (n < 2 || in.angle_increment == 0.0f || in.time_increment == 0.0f)
    {
      pub_->publish(out);
      return;
    }

    // Reset all output ranges to +inf; we'll fill in valid samples.
    const float inf_f = std::numeric_limits<float>::infinity();
    std::fill(out.ranges.begin(), out.ranges.end(), inf_f);
    if (!out.intensities.empty() && out.intensities.size() == out.ranges.size())
    {
      std::fill(out.intensities.begin(), out.intensities.end(), 0.0f);
    }

    const double omega = latest_omega_z_;
    // Reference offset: for "end" the last ray (i = n-1) has dt = 0;
    // for "start" the first ray (i = 0) has dt = 0.
    const double ref_offset_s =
        (reference_ == "start") ? 0.0 : static_cast<double>(in.time_increment) * (n - 1);

    // The angle_increment can be negative (CW scan); handle either way.
    const float ang_min = in.angle_min;
    const float ang_inc = in.angle_increment;
    const float inv_ang_inc = 1.0f / ang_inc;

    for (size_t i = 0; i < n; ++i)
    {
      const float r = in.ranges[i];
      if (!std::isfinite(r) || r < in.range_min || r > in.range_max)
      {
        continue;
      }

      const double dt = static_cast<double>(in.time_increment) * i - ref_offset_s;
      // Geometric: at ray-time the lidar had orientation θ_i, at scan-
      // stamp time it has θ_ref = θ_i + ω·(t_ref - t_i). The ray's
      // endpoint angle expressed in the scan-stamp lidar frame is
      // shifted by -(θ_ref - θ_i) = -ω·(t_ref - t_i) = +ω·dt.
      const double alpha = static_cast<double>(ang_min) +
                           static_cast<double>(ang_inc) * static_cast<double>(i);
      const double alpha_corr = alpha + omega * dt;

      // Re-bin into the output grid (nearest bin).
      const int bin = static_cast<int>(
          std::lround((alpha_corr - static_cast<double>(ang_min)) *
                      static_cast<double>(inv_ang_inc)));
      if (bin < 0 || bin >= static_cast<int>(n))
      {
        continue;
      }

      // Multi-write resolution: keep the nearest range (most pessimistic
      // for collision_monitor — better safe than sorry).
      if (r < out.ranges[bin])
      {
        out.ranges[bin] = r;
        if (!in.intensities.empty() && i < in.intensities.size() &&
            !out.intensities.empty() && bin < static_cast<int>(out.intensities.size()))
        {
          out.intensities[bin] = in.intensities[i];
        }
      }
    }

    pub_->publish(out);
    pub_count_++;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  double latest_omega_z_{0.0};
  rclcpp::Time latest_imu_t_{0, 0, RCL_ROS_TIME};
  bool have_imu_{false};
  std::string reference_{"end"};
  double imu_max_age_s_{0.5};
  size_t pub_count_{0};
  size_t skipped_count_{0};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::ScanDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
