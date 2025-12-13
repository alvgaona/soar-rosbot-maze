#pragma once

#include <memory>
#include <string>
#include <optional>

#include "soar_ros/Subscriber.hpp"
#include "soar_rosbot_msgs/msg/aruco_detection.hpp"

namespace soar_rosbot_controller
{

class ArUcoSubscriber : public soar_ros::Subscriber<soar_rosbot_msgs::msg::ArucoDetection>
{
public:
  ArUcoSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<soar_rosbot_msgs::msg::ArucoDetection>(agent, node, topic),
    aruco_wme_(nullptr),
    detected_wme_(nullptr),
    distance_wme_(nullptr)
  {}

  // Override process_r2s to drain queue and use only the latest value
  void process_r2s() override
  {
    // Drain the queue, keeping only the latest message
    std::optional<soar_rosbot_msgs::msg::ArucoDetection> latest;
    auto res = this->m_r2sQueue.tryPop();
    while (res.has_value()) {
      latest = res;
      res = this->m_r2sQueue.tryPop();
    }

    if (!latest.has_value()) {
      return;  // No messages
    }

    bool detected = latest.value().detected;
    float distance = latest.value().distance;

    sml::Identifier * il = this->m_pAgent->GetInputLink();

    RCLCPP_INFO(
      this->m_node->get_logger(),
      "ArUcoSubscriber: detected=%d, distance=%.3f",
      detected ? 1 : 0,
      distance
    );

    // Remove old WMEs if they exist
    if (aruco_wme_ != nullptr) {
      aruco_wme_->DestroyWME();
      aruco_wme_ = nullptr;
      detected_wme_ = nullptr;
      distance_wme_ = nullptr;
    }

    // Create new WMEs
    aruco_wme_ = il->CreateIdWME("aruco");
    detected_wme_ = aruco_wme_->CreateIntWME("detected", detected ? 1 : 0);
    distance_wme_ = aruco_wme_->CreateFloatWME("distance", distance);
  }

  void parse(soar_rosbot_msgs::msg::ArucoDetection msg) override
  {
    // Not used - we override process_r2s directly
    (void)msg;
  }

private:
  sml::Identifier* aruco_wme_;
  sml::WMElement* detected_wme_;
  sml::WMElement* distance_wme_;
};

}  // namespace soar_rosbot_controller
