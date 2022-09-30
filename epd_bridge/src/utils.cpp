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

std::vector<robot_control_interface_msgs::msg::Object> get_obj_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name, float camera_height)
{ 
    double ppx, ppy, fx, fy;
    ppx = epd_output.ppx;
    ppy = epd_output.ppy;
    fx = epd_output.fx;
    fy = epd_output.fy;
    
    std::vector<robot_control_interface_msgs::msg::Object> objects;
    int idx=0;
    for (auto m : epd_output.objects) {
        if ( m.name == "tray" ) continue;
        robot_control_interface_msgs::msg::Object obj;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = camera_name+"_color_optical_frame";
        
        double angle = atan2(m.axis.y,m.axis.x);
        //double angle = atan2(m.axis.x,m.axis.y);
        //angle = fmod(angle+90,180);
        tf2::Quaternion q;
        //q.setRPY(0, 0, M_PI_2-abs(angle));
        q.setRPY(0, 0, angle);
        //std::cout << "Object [" << idx << "]: angle ->" << angle << std::endl;
        //std::cout << "Object [" << idx << "]: y-vector ->" << m.axis.y << std::endl;
        //std::cout << "Object [" << idx << "]: x-vector ->" << m.axis.x << std::endl;
        
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        //pose.pose.position.x = m.centroid.x;
        //pose.pose.position.y = m.centroid.y;
        //pose.pose.position.z = m.centroid.z;
          
        if (object_name == "BOX") {
            //pose.pose.position.z += 0.05 - 0.0353/2;
            
            //pose.pose.position.x += 0.015;
            //pose.pose.position.y -= 0.005;
            //pose.pose.position.z = 0.55 - 0.0353/2 + 0.02;
            
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            
            if (camera_height < 0.55) {
                pose.pose.position.z = camera_height - 0.04/2 + 0.01;
            } else {
                pose.pose.position.z = camera_height - 0.04/2 - 0.02 + 0.015;
            }
        }

        if (object_name == "LABEL") {
            //pose.pose.position.x += 0.11;
            //pose.pose.position.z += 0.025;
            
            //pose.pose.position.x += 0.01; 
            //pose.pose.position.y -= 0.01;
            //pose.pose.position.z = 0.33;
            
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
        
            if (camera_height < 0.3) {
                pose.pose.position.z = camera_height - 0.04;
            } else {
                pose.pose.position.z = camera_height;
            }
        }

        float obj_surface_depth = m.centroid.z - m.height/2;
        float px = m.centroid.x / obj_surface_depth * fx + ppx;
        float py = m.centroid.y / obj_surface_depth * fy + ppy;

        float x = (px - ppx) / fx * pose.pose.position.z;
        float y = (py - ppy) / fy * pose.pose.position.z;

        pose.pose.position.x += x;
        pose.pose.position.y += y;
        
        shape_msgs::msg::SolidPrimitive shape;
        shape.type = 1;
        shape.dimensions.insert(shape.dimensions.end(), { m.length, m.breadth, m.height });

        if (object_name == "BOX") {
            shape.dimensions[0] = 0.121;
            shape.dimensions[1] = 0.066;
            shape.dimensions[2] = 0.04;
        }

        if (object_name == "LABEL") {
            shape.dimensions[0] = 0.1;
            shape.dimensions[1] = 0.05;
            shape.dimensions[2] = 0.001;
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

std::vector<robot_control_interface_msgs::msg::Box> get_box_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name, float camera_height)
{
    double ppx, ppy, fx, fy;
    ppx = epd_output.ppx;
    ppy = epd_output.ppy;
    fx = epd_output.fx;
    fy = epd_output.fy;
    
    std::vector<robot_control_interface_msgs::msg::Box> boxes;
    int idx=0;
    for (auto m : epd_output.objects) { 
        if ( m.name != "tray" ) continue;
        robot_control_interface_msgs::msg::Box box;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = camera_name+"_color_optical_frame";

        double angle = atan2(m.axis.y,m.axis.x);
        tf2::Quaternion q;
        //q.setRPY(0, 0, M_PI_2-abs(angle));
        q.setRPY(0, 0, angle);
        //std::cout << "Object [" << idx << "]: angle ->" << angle << std::endl;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        //pose.pose.position.x = m.centroid.x + 0.02;
        //pose.pose.position.z = m.centroid.z;
        //pose.pose.position.z = 0.55 - 0.1/2 + 0.06;
        
        //pose.pose.position.x = m.centroid.x;
        //pose.pose.position.y = m.centroid.y;
        //pose.pose.position.z = 0.55 - 0.1/2 + 0.04;

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        //pose.pose.position.z = camera_height - 0.1/2 - 0.02;
        pose.pose.position.z = camera_height - 0.1/2;
        
        float obj_surface_depth = m.centroid.z - m.height/2;
        float px = m.centroid.x / obj_surface_depth * fx + ppx;
        float py = m.centroid.y / obj_surface_depth * fy + ppy;

        float x = (px - ppx) / fx * pose.pose.position.z;
        float y = (py - ppy) / fy * pose.pose.position.z;

        pose.pose.position.x += x;
        pose.pose.position.y += y;
        
        box.box_pose = pose;
        //box.box_dimensions.insert(box.box_dimensions.end(), { m.length, m.breadth, m.height });
        //box.box_dimensions.insert(box.box_dimensions.end(), { m.length, m.breadth, 0.1 });
        box.box_dimensions.insert(box.box_dimensions.end(), { 0.33, 0.33, 0.1 });
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

