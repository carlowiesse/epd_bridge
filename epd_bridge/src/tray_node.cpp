#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision_service_tray"), camera_height(0.0)
  {
    subscription_ = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
    "/easy_perception_deployment/epd_localize_output_tray", 1, std::bind(&VisionNode::epd_callback, this, _1));
    
    service_ = this->create_service<robot_control_interface_msgs::srv::DetectBox>(
    "detect_tray", std::bind(&VisionNode::detect, this, _1, _2));
  }

private:

  void epd_callback(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg)
  {
    objects.clear();
    
    std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg->depth_image, "bgr8");
    frame = imgptr->image;
    
    if (msg->objects.size() > 0) {
      epd_msgs::msg::EPDObjectLocalization epd_output = *msg;
      std::vector<robot_control_interface_msgs::msg::Box> new_objects = get_box_list(epd_output, "pack_camera", "TRAY", camera_height);
      objects.insert(objects.end(), new_objects.begin(), new_objects.end());
    }
  }
  
  void detect(const std::shared_ptr<robot_control_interface_msgs::srv::DetectBox::Request> request, std::shared_ptr<robot_control_interface_msgs::srv::DetectBox::Response> response)
  {
    camera_height = request->camera_height;
    RCLCPP_INFO(this->get_logger(), "Number of objects detected: %d", objects.size());

    std::string timestamp = return_unix_timestamp();
    auto id = std::stoll(timestamp);
    std::string str_id = std::to_string(id);  
    cv::imwrite("tray_"+str_id+".png",frame);
    
    
    if (objects.size() > 0) {
      response->success = true;
      for (int i=0; i < (int)objects.size(); i++) {
        //objects[i].box_pose.pose.position.z = request->camera_height - 0.1/2;
        std::cout << "Object " << i << " [" << objects[i].id << "]" << std::endl;
        std::cout << "Orientation (quaternion) -> x: " << objects[i].box_pose.pose.orientation.x << ", y: "<< objects[i].box_pose.pose.orientation.y << ", z: " << objects[i].box_pose.pose.orientation.z << ", w: " << objects[i].box_pose.pose.orientation.w << std::endl;
        std::cout << "Position (x,y,z) -> x: " << objects[i].box_pose.pose.position.x << ", y: " << objects[i].box_pose.pose.position.y << ", z: " << objects[i].box_pose.pose.position.z << std::endl;
        std::cout << "Dimensions (l,w,h) -> l: " << objects[i].box_dimensions[0] << ", w: " << objects[i].box_dimensions[1] << ", h: " << objects[i].box_dimensions[2] << std::endl;
        std::cout << "Provided camera_height: " << request->camera_height << std::endl;
        std::cout << "---" << std::endl;
      }
      response->boxes = objects;
    }
  }

  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr subscription_;
  rclcpp::Service<robot_control_interface_msgs::srv::DetectBox>::SharedPtr service_;
  std::vector<robot_control_interface_msgs::msg::Box> objects;
  cv::Mat frame;
  float camera_height;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
