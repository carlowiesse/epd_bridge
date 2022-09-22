#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision_service_tray")
  {
    subscription_ = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
    "/easy_perception_deployment/epd_localize_output_tray", 10, std::bind(&VisionNode::epd_callback, this, _1));
    
    service_ = this->create_service<robot_control_interface_msgs::srv::DetectBox>(
    "detect_tray", std::bind(&VisionNode::detect, this, _1, _2));
  }

private:

  void epd_callback(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg)
  {
    objects.clear();
    if (msg->objects.size() > 0) {
      epd_msgs::msg::EPDObjectLocalization epd_output = *msg;
      std::vector<robot_control_interface_msgs::msg::Box> new_objects = get_box_list(epd_output, "pack_camera", "TRAY");
      objects.insert(objects.end(), new_objects.begin(), new_objects.end());
      
      //std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg->depth_image, msg->depth_image.encoding);
      std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg->depth_image, "bgr8");
      frame = imgptr->image;
    }
  }
  
  void detect(const std::shared_ptr<robot_control_interface_msgs::srv::DetectBox::Request> request, std::shared_ptr<robot_control_interface_msgs::srv::DetectBox::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Number of objects detected: %d", objects.size());

    if (objects.size() > 0) {
      
      std::string timestamp = return_unix_timestamp();
      auto id = std::stoll(timestamp);
      std::string str_id = std::to_string(id);  
      cv::imwrite("tray_"+str_id+".png",frame);

      response->success = true;
      response->boxes = objects;
    }
  }

  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr subscription_;
  rclcpp::Service<robot_control_interface_msgs::srv::DetectBox>::SharedPtr service_;
  std::vector<robot_control_interface_msgs::msg::Box> objects;
  cv::Mat frame;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
