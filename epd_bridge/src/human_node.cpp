#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision_service_human"), previous_state(false), current_state(false)
  {
    subscription_ = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
    "/easy_perception_deployment/epd_localize_output_human", 1, std::bind(&VisionNode::epd_callback, this, _1));
    
    client_ = this->create_client<emd_msgs::srv::Operationsrv>(
    "ur_pause");
  }

private:

  void epd_callback(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg)
  {
    std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg->depth_image, "bgr8");
    frame = imgptr->image;
    
    current_state = false;
    
    if (msg->objects.size() > 0) {
      epd_msgs::msg::EPDObjectLocalization epd_output = *msg;
      
      current_state = find_human(epd_output);
      //std::cout << current_state << std::endl;
      //std::cout << previous_state << std::endl;
      //std::cout << "---" << std::endl;
    }

    if (previous_state != current_state) {

      std::string timestamp = return_unix_timestamp();
      auto id = std::stoll(timestamp);
      std::string str_id = std::to_string(id);
      cv::imwrite("human_"+str_id+".png",frame);

      auto request = std::make_shared<emd_msgs::srv::Operationsrv::Request>();
      request->actiongrp = "ur";
      if (current_state) {
        request->operation = "pause";
        RCLCPP_INFO(this->get_logger(), "Human detected ! Requesting to pause ...");
      } else {
        request->operation = "resume";
        RCLCPP_INFO(this->get_logger(), "No human detected ! Requesting to resume ...");
      }

      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto result = client_->async_send_request(request);

    }
    
    previous_state = current_state;
  }
  
  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr subscription_;
  rclcpp::Client<emd_msgs::srv::Operationsrv>::SharedPtr client_;
  bool previous_state;
  bool current_state;
  cv::Mat frame;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
