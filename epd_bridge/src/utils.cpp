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

cv::Mat getCluster(int idxCluster, cv::Mat labels, cv::Mat centers, int nrows)
{
    int colorCluster = (int)centers.at<float>(idxCluster,0);
    cv::Mat mList(labels.size(), CV_8U);
    for(int i=0; i<labels.rows; i++)
    {
        int idx = labels.at<int>(i,0);
        int value = (int)centers.at<float>(idx,0);
        if (value == colorCluster) {
            mList.at<int>(i,0) = 255;
        } else {
            mList.at<int>(i,0) = 0;
        }
    }
    cv::Mat mask = mList.reshape(0,nrows);
    return mask;
}

std::vector<cv::Point> getMaxContour(cv::Mat mask)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    double newArea, maxArea = 0;
    int maxAreaContourId = -1;
    for( size_t i = 0; i < contours.size(); i++ )
    {
        newArea = cv::contourArea(contours.at(i));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = i;
        }
    }
    std::vector<cv::Point> maxContour = contours.at(maxAreaContourId);
    return maxContour;
}

std::vector<robot_control_interface_msgs::msg::Object> get_obj_list(epd_msgs::msg::EPDObjectLocalization epd_output, std::string camera_name, std::string object_name, float camera_height, cv::Mat image_lv0)
{
    double ppx, ppy, fx, fy;
    ppx = epd_output.ppx;
    ppy = epd_output.ppy;
    fx = epd_output.fx;
    fy = epd_output.fy;

    int IMAGE_WIDTH = 640, IMAGE_HEIGHT = 480;
    cv::Mat image_lv1(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8U);
    cv::Mat image_lv2(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8U);
    cv::Mat image_lv3(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8U);
    cv::Mat image_lv4(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8U);


    cv::cvtColor(image_lv0,image_lv1,cv::COLOR_BGR2HSV);
    cv::Mat channels[3];
    cv::split(image_lv1, channels);
    image_lv1 = channels[2];


    int nclusters = 3;
    cv::Mat klabels, centers;
    cv::Mat src = image_lv1.reshape(0, image_lv1.total());
    src.convertTo(src, CV_32F);
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 10, 1.0);
    cv::kmeans(src, nclusters, klabels, criteria, 10, cv::KMEANS_RANDOM_CENTERS, centers);


    double maxVal, minVal;
    cv::Point maxLoc, minLoc;
    cv::minMaxLoc( centers, &minVal, &maxVal, &minLoc, &maxLoc );

    if (object_name == "BOX") {
      image_lv2 = getCluster(minLoc.y, klabels, centers, image_lv1.rows);
    }
    if (object_name == "LABEL") {
      image_lv2 = getCluster(maxLoc.y, klabels, centers, image_lv1.rows);
    }


    cv::Mat cclabels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(image_lv2, cclabels, stats, centroids);
    std::cout << "Number of objects detected (before filtering): " << numLabels-1 << std::endl;


    int buffer = 3;
    std::vector<int> selection;
    for (int n = 1; n < numLabels; n++)
    {
        int area = stats.at<int>(n,cv::CC_STAT_AREA);
        //std::cout << "Area: " << area << std::endl;
        if (object_name == "BOX") {
          if ( area < 4000 || area > 80000 ) continue; // boxes
        }
        if (object_name == "LABEL") {
          if ( area < 2000 || area > 20000 ) continue; // labels
        }

        int left, right, top, bottom;
        left = stats.at<int>(n,cv::CC_STAT_LEFT);
        right = left + stats.at<int>(n,cv::CC_STAT_WIDTH);
        top = stats.at<int>(n,cv::CC_STAT_TOP);
        bottom = top + stats.at<int>(n,cv::CC_STAT_HEIGHT);
        if ( left < buffer || right > IMAGE_WIDTH - buffer ||
             top < buffer || bottom > IMAGE_HEIGHT - buffer ) continue;

        selection.push_back(n);
    }
    std::cout << "Number of objects detected (after filtering): " << selection.size() << std::endl;


    int idx = 0;
    cv::Mat mask;
    cv::Point2f c;
    cv::RotatedRect rRect;
    std::vector<robot_control_interface_msgs::msg::Object> objects(selection.size());
    image_lv4 = image_lv0.clone();

    for ( int s : selection )
    {
        robot_control_interface_msgs::msg::Object obj;
        geometry_msgs::msg::PoseStamped pose;

        //float box_height = 0.066;
        float box_height = 0.04;
        if (object_name == "BOX") {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;

            if (camera_height < 0.5) {
                pose.pose.position.z = camera_height - box_height/2;
            } else {
                pose.pose.position.z = camera_height - box_height/2 - 0.02;
            }
        }

        if (object_name == "LABEL") {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;

            if (camera_height < 0.3) {
                pose.pose.position.z = camera_height - box_height;
            } else {
                pose.pose.position.z = camera_height;
            }
        }

        c.x = centroids.at<double>(s,0);
        c.y = centroids.at<double>(s,1);

        float x = (c.x - ppx) / fx * pose.pose.position.z;
        float y = (c.y - ppy) / fy * pose.pose.position.z;

        pose.pose.position.x += x;
        pose.pose.position.y += y;

        cv::compare(cclabels, cv::Scalar(s), mask, cv::CMP_EQ);
        std::vector<cv::Point> maxContour = getMaxContour(mask);
        cv::RotatedRect rRect = cv::minAreaRect(maxContour);

        if (object_name == "LABEL") {
          float dx = c.x - rRect.center.x;
          float dy = c.y - rRect.center.y;
          float xdis = -dx/sqrt(dx*dx+dy*dy);
          float ydis = -dy/sqrt(dx*dx+dy*dy);
          pose.pose.position.x += xdis*0.01;
          pose.pose.position.y += ydis*0.01;
        }

        float yaw = 360 - rRect.angle;
        if (rRect.size.width < rRect.size.height) yaw += 90;
        //yaw = fmod(yaw+90,180);
        yaw = -fmod(yaw,180);
        //yaw -= 180;
        //std::cout << "yaw: " << yaw << " deg" << std::endl;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw*(M_PI/180));
        //std::cout << "quaternion: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

        pose.header.frame_id = camera_name+"_color_optical_frame";
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = 1;

        shape.dimensions.insert(shape.dimensions.end(), { 0.0, 0.0, 0.0 });

        if (object_name == "BOX") {
            //shape.dimensions[0] = 0.121;
            //shape.dimensions[1] = 0.066;
            //shape.dimensions[2] = 0.04;
            //shape.dimensions[0] = 0.121;
            //shape.dimensions[1] = 0.04;
            //shape.dimensions[2] = 0.066;
            shape.dimensions[0] = 0.087;
            shape.dimensions[1] = 0.057;
            shape.dimensions[2] = 0.04;
        }

        if (object_name == "LABEL") {
            //shape.dimensions[0] = 0.1;
            //shape.dimensions[1] = 0.05;
            //shape.dimensions[2] = 0.001;
            shape.dimensions[0] = 0.08;
            shape.dimensions[1] = 0.04;
            shape.dimensions[2] = 0.0001;
        }

        obj.pose = pose;
        obj.shape = shape;
        obj.label = object_name;

        std::string timestamp = return_unix_timestamp();
        auto id = std::stoll(timestamp) + idx;
        std::string str_id = std::to_string(id);
        obj.id = object_name+"_"+str_id;

        objects[idx] = obj;
        idx++;

        cv::drawContours(image_lv4, std::vector<std::vector<cv::Point>>{ maxContour }, 0, cv::Scalar(0,0,255), 2);
        cv::circle(image_lv4, c, 4, cv::Scalar(0,0,255), cv::FILLED);
    }
    auto t = return_current_time_and_date();
    cv::imwrite("opencv_"+object_name+"_"+std::to_string(idx)+"_"+t+".png", image_lv4);

    return objects;
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

        //float box_height = 0.066;
        float box_height = 0.04;
        if (object_name == "BOX") {
            //pose.pose.position.z += 0.05 - 0.0353/2;

            //pose.pose.position.x += 0.015;
            //pose.pose.position.y -= 0.005;
            //pose.pose.position.z = 0.55 - 0.0353/2 + 0.02;

            //pose.pose.position.x = -0.01;
            pose.pose.position.x = 0;
            //pose.pose.position.y = 0.015;
            pose.pose.position.y = 0;

            if (camera_height < 0.5) {
                //pose.pose.position.z = camera_height - 0.04/2 + 0.01;
                //pose.pose.position.z = camera_height - 0.04/2;
                pose.pose.position.z = camera_height - box_height/2;
            } else {
                //pose.pose.position.z = camera_height - 0.04/2 - 0.02 + 0.015;
                //pose.pose.position.z = camera_height - 0.04/2 - 0.02;
                pose.pose.position.z = camera_height - box_height/2 - 0.02;
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
                //pose.pose.position.z = camera_height - 0.04;
                pose.pose.position.z = camera_height - box_height;
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
            //shape.dimensions[0] = 0.121;
            //shape.dimensions[1] = 0.066;
            //shape.dimensions[2] = 0.04;
            //shape.dimensions[0] = 0.121;
            //shape.dimensions[1] = 0.04;
            //shape.dimensions[2] = 0.066;
            shape.dimensions[0] = 0.087;
            shape.dimensions[1] = 0.057;
            shape.dimensions[2] = 0.04;
        }

        if (object_name == "LABEL") {
            //shape.dimensions[0] = 0.1;
            //shape.dimensions[1] = 0.05;
            //shape.dimensions[2] = 0.001;
            shape.dimensions[0] = 0.08;
            shape.dimensions[1] = 0.04;
            shape.dimensions[2] = 0.0001;
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

