#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace soar_rosbot_controller
{

/**
 * @brief Simple test node that observes and publishes robot yaw orientation from TF
 *
 * Reads the odom->base_link transform and publishes the yaw angle
 * to /robot/yaw at 10 Hz.
 */
class YawObserver : public rclcpp::Node
{
public:
  YawObserver();
  ~YawObserver() = default;

private:
  /**
   * @brief Timer callback to publish yaw from TF
   */
  void publishYaw();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace soar_rosbot_controller
