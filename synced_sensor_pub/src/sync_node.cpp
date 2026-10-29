// Approximate-time synchronizer for:
//   - ZED X compressed depth  (sensor_msgs/CompressedImage, compressedDepth)
//   - ZED X compressed RGB    (sensor_msgs/CompressedImage)
//   - Ouster point cloud      (sensor_msgs/PointCloud2)
// and republishes the three messages together under /synced/... topics.
// Also publishes synchronized RGB and depth CameraInfo messages.

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::CameraInfo;

class SyncNode : public rclcpp::Node
{
public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    CompressedImage, CompressedImage, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  SyncNode() : Node("sync_node")
  {
    // Input topics
    const auto depth_in = declare_parameter<std::string>(
      "depth_topic", "/zedx_front/zed_node/depth/depth_registered/compressedDepth");
    const auto rgb_in = declare_parameter<std::string>(
      "rgb_topic", "/zedx_front/zed_node/rgb/color/rect/image/compressed");
    const auto lidar_in = declare_parameter<std::string>(
      "lidar_topic", "/ouster/points");

    // Output topics
    const auto depth_out = declare_parameter<std::string>(
      "synced_depth_topic", "/synced/depth/compressedDepth");
    const auto rgb_out = declare_parameter<std::string>(
      "synced_rgb_topic", "/synced/rgb/compressed");
    const auto lidar_out = declare_parameter<std::string>(
      "synced_lidar_topic", "/synced/points");

    // CameraInfo output topics
    const auto depth_info_out = declare_parameter<std::string>(
      "synced_depth_camera_info_topic", "/synced/depth/camera_info");
    const auto rgb_info_out = declare_parameter<std::string>(
      "synced_rgb_camera_info_topic", "/synced/rgb/camera_info");

    // Sync tuning
    const int queue_size = declare_parameter<int>("queue_size", 10);
    const double slop = declare_parameter<double>("slop", 0.05);
    // If true, all three original outputs get the RGB header stamp.
    // CameraInfo messages will also use the RGB header stamp.
    restamp_ = declare_parameter<bool>("restamp_to_rgb", false);

    // Sensor drivers normally publish best-effort; match that on the inputs.
    const auto qos = rmw_qos_profile_sensor_data;
    depth_sub_.subscribe(this, depth_in, qos);
    rgb_sub_.subscribe(this, rgb_in, qos);
    lidar_sub_.subscribe(this, lidar_in, qos);

    // Outputs are reliable so image_transport republish (reliable by default) connects.
    const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    depth_pub_ = create_publisher<CompressedImage>(depth_out, out_qos);
    rgb_pub_ = create_publisher<CompressedImage>(rgb_out, out_qos);
    lidar_pub_ = create_publisher<PointCloud2>(lidar_out, out_qos);

    // CameraInfo publishers
    depth_info_pub_ = create_publisher<CameraInfo>(depth_info_out, out_qos);
    rgb_info_pub_ = create_publisher<CameraInfo>(rgb_info_out, out_qos);

    // Initialize camera calibration information.
    // The calibration values are identical for depth and RGB.
    camera_info_template_.header.frame_id =
      "zedx_front_left_camera_frame_optical";

    camera_info_template_.height = 540;
    camera_info_template_.width = 960;
    camera_info_template_.distortion_model = "rational_polynomial";

    camera_info_template_.d = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    camera_info_template_.k = {
      363.8719482421875, 0.0, 483.87677001953125,
      0.0, 363.8719482421875, 270.46142578125,
      0.0, 0.0, 1.0
    };

    camera_info_template_.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    camera_info_template_.p = {
      363.8719482421875, 0.0, 483.87677001953125, 0.0,
      0.0, 363.8719482421875, 270.46142578125, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    camera_info_template_.binning_x = 0;
    camera_info_template_.binning_y = 0;

    camera_info_template_.roi.x_offset = 0;
    camera_info_template_.roi.y_offset = 0;
    camera_info_template_.roi.height = 0;
    camera_info_template_.roi.width = 0;
    camera_info_template_.roi.do_rectify = false;

    // Synchronizer
    sync_ = std::make_shared<Sync>(
      SyncPolicy(queue_size), depth_sub_, rgb_sub_, lidar_sub_);

    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));

    sync_->registerCallback(
      std::bind(&SyncNode::callback, this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

    RCLCPP_INFO(
      get_logger(),
      "Syncing (slop=%.3fs, queue=%d):\n  %s\n  %s\n  %s\n"
      "CameraInfo outputs:\n  %s\n  %s",
      slop, queue_size,
      depth_in.c_str(), rgb_in.c_str(), lidar_in.c_str(),
      depth_info_out.c_str(), rgb_info_out.c_str());
  }

private:
  void callback(
    const CompressedImage::ConstSharedPtr & depth,
    const CompressedImage::ConstSharedPtr & rgb,
    const PointCloud2::ConstSharedPtr & lidar)
  {
    // Preserve the original publishing behavior.
    if (!restamp_) {
      depth_pub_->publish(*depth);
      rgb_pub_->publish(*rgb);
      lidar_pub_->publish(*lidar);
    } else {
      const auto stamp = rgb->header.stamp;

      auto d = std::make_unique<CompressedImage>(*depth);
      auto r = std::make_unique<CompressedImage>(*rgb);
      auto l = std::make_unique<PointCloud2>(*lidar);

      d->header.stamp = stamp;
      l->header.stamp = stamp;

      depth_pub_->publish(std::move(d));
      rgb_pub_->publish(std::move(r));
      lidar_pub_->publish(std::move(l));
    }

    // Publish CameraInfo alongside the synchronized images.
    // Use the RGB timestamp so downstream consumers can associate
    // both CameraInfo messages with the synchronized RGB frame.
    auto depth_info = camera_info_template_;
    auto rgb_info = camera_info_template_;

    depth_info.header.stamp = rgb->header.stamp;
    rgb_info.header.stamp = rgb->header.stamp;

    depth_info_pub_->publish(depth_info);
    rgb_info_pub_->publish(rgb_info);
  }

  bool restamp_{false};

  message_filters::Subscriber<CompressedImage> depth_sub_;
  message_filters::Subscriber<CompressedImage> rgb_sub_;
  message_filters::Subscriber<PointCloud2> lidar_sub_;

  std::shared_ptr<Sync> sync_;

  rclcpp::Publisher<CompressedImage>::SharedPtr depth_pub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr lidar_pub_;

  rclcpp::Publisher<CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr rgb_info_pub_;

  CameraInfo camera_info_template_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncNode>());
  rclcpp::shutdown();
  return 0;
}
