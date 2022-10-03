#ifndef UTILS_HPP
#define UTILS_HPP

#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/image.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "shape_msgs/msg/solid_primitive.hpp"

#include "epd_msgs/msg/localized_object.hpp"
#include "epd_msgs/msg/epd_object_localization.hpp"

#include "ur_msgs/srv/operationsrv.hpp"

#include "emd_msgs/srv/operationsrv.hpp"

#include "robot_control_interface_msgs/msg/box.hpp"
#include "robot_control_interface_msgs/msg/object.hpp"

#include "robot_control_interface_msgs/srv/detect_box.hpp"
#include "robot_control_interface_msgs/srv/detect_objects.hpp"

using namespace std::chrono_literals;

std::string return_unix_timestamp();

std::string return_current_time_and_date();

cv::Mat getCluster(int idxCluster, cv::Mat labels, cv::Mat centers, int nrows);

std::vector<cv::Point> getMaxContour(cv::Mat mask);

std::vector<robot_control_interface_msgs::msg::Object> get_obj_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name, float camera_height, cv::Mat image_lv0);

std::vector<robot_control_interface_msgs::msg::Object> get_obj_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name, float camera_height);

std::vector<robot_control_interface_msgs::msg::Box> get_box_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name, float camera_height);

bool find_human(epd_msgs::msg::EPDObjectLocalization epd_output);

#endif // UTILS_HPP
