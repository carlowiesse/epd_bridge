#include "utils.hpp"

std::string return_unix_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    auto chrono_t = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    std::string str_t = std::to_string(chrono_t);
    return str_t;
}

std::string return_current_time_and_date()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H:%M:%S");
    return ss.str();
}

std::vector<robot_control_interface_msgs::msg::Object> get_obj_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name)
{ 
    std::vector<robot_control_interface_msgs::msg::Object> objects;
    int idx=0;
    for (auto m : epd_output.objects) {
        robot_control_interface_msgs::msg::Object obj;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = camera_name+"_color_optical_frame";
        
        tf2::Quaternion q;
        //q.setRPY(m.axis.x, m.axis.y, m.axis.z);
        //q.setRPY(m.axis.x, 0, m.axis.z+M_PI_2);
        q.setRPY(m.axis.x, 0, m.axis.z);
        //std::cout << m.axis.z << std::endl;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose.pose.position.x = m.centroid.x;
        pose.pose.position.y = m.centroid.y;
        pose.pose.position.z = m.centroid.z;

        if (object_name == "BOX") {
            pose.pose.position.z += 0.0353/2;
        }

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = 1;
        shape.dimensions.insert(shape.dimensions.end(), { m.length, m.breadth, m.height });

        if (object_name == "BOX") {
            shape.dimensions[2] = 0.0353;
        }
        
        obj.pose = pose;
        obj.shape = shape;
        obj.label = object_name;

        std::string timestamp = return_unix_timestamp();
        auto id = std::stoll(timestamp) + idx;
        std::string str_id = std::to_string(id);
        obj.id = object_name+"_"+str_id;

        objects.push_back(obj);
        idx++;
    }
    return objects;
}

std::vector<robot_control_interface_msgs::msg::Box> get_box_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name)
{
    std::vector<robot_control_interface_msgs::msg::Box> boxes;
    int idx=0;
    for (auto m : epd_output.objects) {
        robot_control_interface_msgs::msg::Box box;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = camera_name+"_color_optical_frame";

        tf2::Quaternion q;
        q.setRPY(m.axis.x, m.axis.y, m.axis.z);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose.pose.position.x = m.centroid.x;
        pose.pose.position.y = m.centroid.y;
        //pose.pose.position.z = m.centroid.z;
        pose.pose.position.z = m.centroid.z + 0.1/2;

        box.box_pose = pose;
        //box.box_dimensions.insert(box.box_dimensions.end(), { m.length, m.breadth, m.height });
        box.box_dimensions.insert(box.box_dimensions.end(), { m.length, m.breadth, 0.1 });
        box.thickness = 0.02;

        std::string timestamp = return_unix_timestamp();
        auto id = std::stoll(timestamp) + idx;
        std::string str_id = std::to_string(id);
        box.id = object_name+"_"+str_id;

        boxes.push_back(box);
        idx++;
    }
    return boxes;
}

bool find_human(epd_msgs::msg::EPDObjectLocalization epd_output)
{
    for (auto m : epd_output.objects) {
        if ( m.name == "person" ) return true;
    }
    return false;
}

