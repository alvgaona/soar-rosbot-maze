#ifndef SOAR_ROSBOT_CONTROLLER_PERCEPTION_SUBSCRIBERS_HPP_
#define SOAR_ROSBOT_CONTROLLER__PERCEPTION_SUBSCRIBERS_HPP_

#include <memory>
#include <string>
#include <optional>

#include "soar_ros/Subscriber.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace soar_rosbot_controller
{

class ArUcoDetectedSubscriber : public soar_ros::Subscriber<std_msgs::msg::Bool>
{
public:
  ArUcoDetectedSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<std_msgs::msg::Bool>(agent, node, topic),
    aruco_wme_(nullptr),
    detected_wme_(nullptr)
  {}

  // Override process_r2s to drain queue and use only the latest value
  void process_r2s() override
  {
    // Drain the queue, keeping only the latest message
    std::optional<std_msgs::msg::Bool> latest;
    auto res = this->m_r2sQueue.tryPop();
    while (res.has_value()) {
      latest = res;
      res = this->m_r2sQueue.tryPop();
    }

    if (!latest.has_value()) {
      return;  // No messages
    }

    bool detected = latest.value().data;

    sml::Identifier * il = this->m_pAgent->GetInputLink();

    RCLCPP_INFO(this->m_node->get_logger(), "ArUcoDetectedSubscriber using latest detected=%d", detected ? 1 : 0);

    // Remove old WMEs if they exist
    if (aruco_wme_ != nullptr) {
      aruco_wme_->DestroyWME();
      aruco_wme_ = nullptr;
      detected_wme_ = nullptr;
    }

    // Create new WMEs
    aruco_wme_ = il->CreateIdWME("aruco");
    detected_wme_ = aruco_wme_->CreateIntWME("detected", detected ? 1 : 0);
  }

  void parse(std_msgs::msg::Bool msg) override
  {
    // Not used - we override process_r2s directly
    (void)msg;
  }

private:
  sml::Identifier* aruco_wme_;
  sml::WMElement* detected_wme_;
};

class ArUcoDistanceSubscriber : public soar_ros::Subscriber<std_msgs::msg::Float32>
{
public:
  ArUcoDistanceSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<std_msgs::msg::Float32>(agent, node, topic),
    aruco_wme_(nullptr),
    distance_wme_(nullptr)
  {}

  void parse(std_msgs::msg::Float32 msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();

    // Remove old WMEs if they exist
    if (aruco_wme_ != nullptr) {
      aruco_wme_->DestroyWME();
      aruco_wme_ = nullptr;
      distance_wme_ = nullptr;
    }

    // Create new WMEs
    aruco_wme_ = il->CreateIdWME("aruco");
    distance_wme_ = aruco_wme_->CreateFloatWME("distance", msg.data);
  }

private:
  sml::Identifier* aruco_wme_;
  sml::WMElement* distance_wme_;
};

class WallSubscriber : public soar_ros::Subscriber<std_msgs::msg::Bool>
{
public:
  WallSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic,
    const std::string & direction)
  : Subscriber<std_msgs::msg::Bool>(agent, node, topic),
    direction_(direction),
    walls_wme_(nullptr)
  {}

  void parse(std_msgs::msg::Bool msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();

    walls_wme_ = il->CreateIdWME("walls");
    walls_wme_->CreateIntWME(direction_.c_str(), msg.data ? 1 : 0);
  }

private:
  std::string direction_;
  sml::Identifier * walls_wme_;
};

}  // namespace soar_rosbot_controller

#endif  // SOAR_ROSBOT_CONTROLLER__PERCEPTION_SUBSCRIBERS_HPP_
