#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision_service_box_tray")
  {
    subscription_ = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
    "/easy_perception_deployment/epd_localize_output_box_tray", 1, std::bind(&VisionNode::epd_callback, this, _1));
    
    service_box = this->create_service<robot_control_interface_msgs::srv::DetectObjects>(
    "detect_box", std::bind(&VisionNode::detect_box, this, _1, _2));
    
    service_tray = this->create_service<robot_control_interface_msgs::srv::DetectBox>(
    "detect_tray", std::bind(&VisionNode::detect_tray, this, _1, _2));
  }

private:

  void epd_callback(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg)
  {
    box_objects.clear();
    tray_objects.clear();
    
    std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg->depth_image, "bgr8");
    frame = imgptr->image;
    
    if (msg->objects.size() > 0) {
      epd_msgs::msg::EPDObjectLocalization epd_output = *msg;
      
      std::vector<robot_control_interface_msgs::msg::Object> new_box_objects = get_obj_list(epd_output, "pack_camera", "BOX");
      box_objects.insert(box_objects.end(), new_box_objects.begin(), new_box_objects.end());
      
      std::vector<robot_control_interface_msgs::msg::Box> new_tray_objects = get_box_list(epd_output, "pack_camera", "TRAY");
      tray_objects.insert(tray_objects.end(), new_tray_objects.begin(), new_tray_objects.end());
    }
  }
  
  void detect_box(const std::shared_ptr<robot_control_interface_msgs::srv::DetectObjects::Request> request, std::shared_ptr<robot_control_interface_msgs::srv::DetectObjects::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Number of box objects detected: %d", box_objects.size());
 
    std::string timestamp = return_unix_timestamp();
    auto id = std::stoll(timestamp);
    std::string str_id = std::to_string(id);  
    cv::imwrite("box_"+str_id+".png",frame);

    std::cout << "Request camera_height: " << request->camera_height << std::endl;
    
    if (box_objects.size() > 0) {
      response->success = true;
      if (request->single_object) {
        float x, y, d, min_d = 1; int winner_idx;
        for (int i=0; i < (int)box_objects.size(); i++) {
          x = box_objects[i].pose.pose.position.x;
          y = box_objects[i].pose.pose.position.y;
          d = sqrtf(x * x + y * y);
          if (d < min_d) {
            min_d = d;
            winner_idx = i;
          }
        }
        std::vector<robot_control_interface_msgs::msg::Object> single_object;
        single_object.push_back(box_objects[winner_idx]);
        single_object[0].pose.pose.position.z = request->camera_height-0.04/2;
        response->objects = single_object;
      } else {
        for (int i=0; i < (int)box_objects.size(); i++)
          box_objects[i].pose.pose.position.z = request->camera_height-0.04/2;
        response->objects = box_objects;
      }
    }
  }
  
  void detect_tray(const std::shared_ptr<robot_control_interface_msgs::srv::DetectBox::Request> request, std::shared_ptr<robot_control_interface_msgs::srv::DetectBox::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Number of tray objects detected: %d", tray_objects.size());

    std::string timestamp = return_unix_timestamp();
    auto id = std::stoll(timestamp);
    std::string str_id = std::to_string(id);  
    cv::imwrite("tray_"+str_id+".png",frame);
    
    std::cout << "Request camera_height: " << request->camera_height << std::endl;
    
    if (tray_objects.size() > 0) {
      response->success = true;
      for (int i=0; i < (int)tray_objects.size(); i++)
        tray_objects[i].box_pose.pose.position.z = request->camera_height - 0.1/2;
      response->boxes = tray_objects;
    }
  }

  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr subscription_;
  rclcpp::Service<robot_control_interface_msgs::srv::DetectObjects>::SharedPtr service_box;
  rclcpp::Service<robot_control_interface_msgs::srv::DetectBox>::SharedPtr service_tray;
  std::vector<robot_control_interface_msgs::msg::Object> box_objects;
  std::vector<robot_control_interface_msgs::msg::Box> tray_objects;
  cv::Mat frame;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
