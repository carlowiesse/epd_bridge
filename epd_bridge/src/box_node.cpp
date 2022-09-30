#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision_service_box"), camera_height(0.0)
  {
    subscription_ = this->create_subscription<epd_msgs::msg::EPDObjectLocalization>(
    "/easy_perception_deployment/epd_localize_output_box", 1, std::bind(&VisionNode::epd_callback, this, _1));
    
    service_ = this->create_service<robot_control_interface_msgs::srv::DetectObjects>(
    "detect_box", std::bind(&VisionNode::detect, this, _1, _2));
  }

private:

  void epd_callback(const epd_msgs::msg::EPDObjectLocalization::SharedPtr msg)
  {
    objects.clear();
    
    std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg->depth_image, "bgr8");
    frame = imgptr->image;
    
    if (msg->objects.size() > 0) {
      epd_msgs::msg::EPDObjectLocalization epd_output = *msg;
      std::vector<robot_control_interface_msgs::msg::Object> new_objects = get_obj_list(epd_output, "pack_camera", "BOX", camera_height);
      objects.insert(objects.end(), new_objects.begin(), new_objects.end());
    }
  }
  
  void detect(const std::shared_ptr<robot_control_interface_msgs::srv::DetectObjects::Request> request, std::shared_ptr<robot_control_interface_msgs::srv::DetectObjects::Response> response)
  {
    camera_height = request->camera_height;
    RCLCPP_INFO(this->get_logger(), "Number of objects detected: %d", objects.size());
 
    std::string timestamp = return_unix_timestamp();
    auto id = std::stoll(timestamp);
    std::string str_id = std::to_string(id);  
    cv::imwrite("box_"+str_id+".png",frame);

    if (objects.size() > 0) {
      response->success = true;
      if (request->single_object) {
        float x, y, d, min_d = 1; int winner_idx;
        for (int i=0; i < (int)objects.size(); i++) {
          x = objects[i].pose.pose.position.x;
          y = objects[i].pose.pose.position.y;
          d = sqrtf(x * x + y * y);
          if (d < min_d) {
            min_d = d;
            winner_idx = i;
          }
        }
        std::vector<robot_control_interface_msgs::msg::Object> single_object;
        single_object.push_back(objects[winner_idx]);
        //if (request->camera_height < 0.55) {
        //  single_object[0].pose.pose.position.z = request->camera_height-0.04/2;
        //} else {
        //  single_object[0].pose.pose.position.z = request->camera_height-0.04/2-0.02;
        //}
        std::cout << "Object 0" << " [" << single_object[0].id << "]" << std::endl;
        std::cout << "Orientation (quaternion) -> x: " << single_object[0].pose.pose.orientation.x << ", y: "<< single_object[0].pose.pose.orientation.y << ", z: " << single_object[0].pose.pose.orientation.z << ", w: " << single_object[0].pose.pose.orientation.w << std::endl;
        std::cout << "Position (x,y,z) -> x: " << single_object[0].pose.pose.position.x << ", y: " << single_object[0].pose.pose.position.y << ", z: " << single_object[0].pose.pose.position.z << std::endl;
        std::cout << "Dimensions (l,w,h) -> l: " << single_object[0].shape.dimensions[0] << ", w: " << single_object[0].shape.dimensions[1] << ", h: " << single_object[0].shape.dimensions[2] << std::endl;
        std::cout << "Provided camera_height: " << request->camera_height << std::endl;
        std::cout << "---" << std::endl;
        response->objects = single_object;
      } else {
        for (int i=0; i < (int)objects.size(); i++) {
          //if (request->camera_height < 0.55) {
          //  objects[i].pose.pose.position.z = request->camera_height-0.04/2+0.01;
          //} else {
          //  objects[i].pose.pose.position.z = request->camera_height-0.04/2-0.02+0.015;
          //}
          std::cout << "Object " << i << " [" << objects[i].id << "]" << std::endl;
          std::cout << "Orientation (quaternion) -> x: " << objects[i].pose.pose.orientation.x << ", y: "<< objects[i].pose.pose.orientation.y << ", z: " << objects[i].pose.pose.orientation.z << ", w: " << objects[i].pose.pose.orientation.w << std::endl;
          std::cout << "Position (x,y,z) -> x: " << objects[i].pose.pose.position.x << ", y: " << objects[i].pose.pose.position.y << ", z: " << objects[i].pose.pose.position.z << std::endl;
          std::cout << "Dimensions (l,w,h) -> l: " << objects[i].shape.dimensions[0] << ", w: " << objects[i].shape.dimensions[1] << ", h: " << objects[i].shape.dimensions[2] << std::endl;
          std::cout << "Provided camera_height: " << request->camera_height << std::endl;
          std::cout << "---" << std::endl;
        }
        response->objects = objects;
      }
    }
  }

  rclcpp::Subscription<epd_msgs::msg::EPDObjectLocalization>::SharedPtr subscription_;
  rclcpp::Service<robot_control_interface_msgs::srv::DetectObjects>::SharedPtr service_;
  std::vector<robot_control_interface_msgs::msg::Object> objects;
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
