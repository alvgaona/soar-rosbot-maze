#include <chrono>
#include <cmath>

#include "soar_rosbot_controller/yaw_observer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace soar_rosbot_controller
{

YawObserver::YawObserver()
: Node("yaw_observer")
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/rosbot/yaw", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() { publishYaw(); });

  RCLCPP_INFO(this->get_logger(), "Yaw observer initialized");
  RCLCPP_INFO(this->get_logger(), "Publishing yaw to /rosbot/yaw");
}

void YawObserver::publishYaw()
{
  try {
    // Get transform from odom to base_link
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);

    // Extract yaw angle from quaternion
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Publish yaw
    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = yaw;
    yaw_pub_->publish(yaw_msg);

    RCLCPP_DEBUG(this->get_logger(), "Yaw: %.3f rad (%.1f deg)", yaw, yaw * 180.0 / M_PI);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,  // 5 seconds
      "Could not get transform odom->base_link: %s", ex.what());
  }
}

}  // namespace soar_rosbot_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<soar_rosbot_controller::YawObserver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
