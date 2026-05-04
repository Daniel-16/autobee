// Monocular ORB-SLAM3 wrapper node for AutoBee.
//
// Subscribes:
//   /drone/camera/image_raw  sensor_msgs/Image
// Publishes:
//   /drone/pose              geometry_msgs/Pose
//
// Uses the ORB-SLAM3 monocular pipeline (System::TrackMonocular). On each
// successful frame, converts the returned Sophus::SE3f camera pose into the
// world frame Pose msg the rest of the AutoBee stack expects.
//
// Why monocular and not monocular-inertial: the inertial pipeline needs a
// well-tuned IMU model and a careful synchronisation of frames + IMU samples.
// For a sim-to-Tello dissertation milestone, a monocular pipeline is the
// pragmatic floor — Tello has a usable mono camera and questionable IMU
// quality, so monocular fits the deployment target.
//
// Coordinate convention: ORB-SLAM3 returns Tcw (camera-from-world). We invert
// to Twc (world-from-camera) so the published pose tracks the drone in the
// fixed world frame.

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cv_bridge/cv_bridge.h>

// ORB-SLAM3 main include
#include "System.h"

namespace
{
constexpr const char *kImageTopic = "/drone/camera/image_raw";
constexpr const char *kPoseTopic = "/drone/pose";
}  // namespace

class OrbSlam3Node : public rclcpp::Node
{
public:
  OrbSlam3Node()
  : rclcpp::Node("orb_slam3_node")
  {
    declare_parameter<std::string>("vocab_path", "");
    declare_parameter<std::string>("settings_path", "");
    declare_parameter<bool>("use_viewer", false);
    declare_parameter<std::string>("image_encoding", "bgr8");

    const std::string vocab = get_parameter("vocab_path").as_string();
    const std::string settings = get_parameter("settings_path").as_string();
    use_viewer_ = get_parameter("use_viewer").as_bool();
    image_encoding_ = get_parameter("image_encoding").as_string();

    if (vocab.empty() || settings.empty()) {
      RCLCPP_FATAL(get_logger(),
        "vocab_path and settings_path parameters are required. "
        "Pass via --ros-args -p vocab_path:=... -p settings_path:=...");
      throw std::runtime_error("missing slam params");
    }

    RCLCPP_INFO(get_logger(), "loading ORB-SLAM3 (vocab=%s settings=%s)",
      vocab.c_str(), settings.c_str());
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocab, settings, ORB_SLAM3::System::MONOCULAR, use_viewer_);

    pose_pub_ = create_publisher<geometry_msgs::msg::Pose>(kPoseTopic, 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      kImageTopic, rclcpp::SensorDataQoS(),
      std::bind(&OrbSlam3Node::OnImage, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "orb_slam3_node up; subscribing %s, publishing %s",
      kImageTopic, kPoseTopic);
  }

  ~OrbSlam3Node() override
  {
    if (slam_) {
      slam_->Shutdown();
    }
  }

private:
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, image_encoding_);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    cv::Mat gray;
    if (cv_ptr->image.channels() == 3) {
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = cv_ptr->image;
    }

    const double t_seconds =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    Sophus::SE3f Tcw;
    {
      std::lock_guard<std::mutex> lock(slam_mutex_);
      Tcw = slam_->TrackMonocular(gray, t_seconds);
    }

    if (Tcw.matrix().isZero(1e-9)) {
      // Tracking lost / not yet initialised -- skip.
      return;
    }

    // World-from-camera pose
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();

    geometry_msgs::msg::Pose pose;
    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_pub_->publish(pose);
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::mutex slam_mutex_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  bool use_viewer_;
  std::string image_encoding_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OrbSlam3Node>());
  } catch (const std::exception & e) {
    fprintf(stderr, "fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
