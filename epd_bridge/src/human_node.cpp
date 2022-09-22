#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision_service_human"), previous_state(false)
  {
    subscription_ = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
    "/easy_perception_deployment/epd_localize_output_human", 10, std::bind(&VisionNode::epd_callback, this, _1));
    
    client_ = this->create_client<emd_msgs::srv::Operationsrv>(
    "detect_human");
  }

private:

  void epd_callback(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "Number of objects detected: %d", msg->objects.size());
    if (msg->objects.size() > 0) {
      epd_msgs::msg::EPDObjectLocalization epd_output = *msg;
      
      bool current_state = find_human(epd_output);
      if (previous_state != current_state) {
        auto request = std::make_shared<emd_msgs::srv::Operationsrv::Request>();
        request->actiongrp = "ur";
        if (current_state) {
          request->operation = "pause";
        } else {
          request->operation = "resume";
        }
      }
      previous_state = current_state;
    }
  }
  
  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr subscription_;
  rclcpp::Client<emd_msgs::srv::Operationsrv>::SharedPtr client_;
  bool previous_state;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
