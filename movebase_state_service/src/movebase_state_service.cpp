#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <string>
#include "movebase_state_service/MovebaseStateService.h"

class state_service {

public:

    ros::NodeHandle n;
    ros::ServiceServer service;

    //sensor_msgs::Image::ConstPtr last_img;
    //nav_msgs::OccupancyGrid::ConstPtr global_last_map;
    //nav_msgs::OccupancyGrid::ConstPtr local_last_map;
    geometry_msgs::PoseStampedConstPtr last_goal;
    geometry_msgs::PoseConstPtr last_pose;
    nav_msgs::Path::ConstPtr global_last_path;
    nav_msgs::Path::ConstPtr local_last_path;
    std::string snapshot_folder;
    int counter;

    ros::Subscriber goal_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber global_path_sub;
    ros::Subscriber local_path_sub;

    std::string image_input;
    std::string global_costmap_input;
    std::string local_costmap_input;
    std::string goal_input;
    std::string pose_input;
    std::string global_path_input;
    std::string local_path_input;

    state_service()
    {
        // Initialize node parameters from launch file or command line.
        // Use a private node handle so that multiple instances of the node can be run simultaneously
        // while using different parameters.
        ros::NodeHandle pn("~");
        pn.param<std::string>("image_input", image_input, std::string("/head_xtion/rgb/image_color"));
        pn.param<std::string>("global_costmap_input", global_costmap_input, std::string("/move_base/global_costmap/costmap"));
        pn.param<std::string>("local_costmap_input", local_costmap_input, std::string("/move_base/local_costmap/costmap"));
        pn.param<std::string>("snapshot_folder", snapshot_folder, std::string(getenv("HOME")) + std::string("/.ros/state_snapshot_service"));
        pn.param<std::string>("goal_input", goal_input, std::string("/move_base/current_goal"));
        pn.param<std::string>("pose_input", pose_input, std::string("/robot_pose"));
        pn.param<std::string>("global_path_input", global_path_input, std::string("/move_base/NavfnROS/plan"));
        pn.param<std::string>("local_path_input", local_path_input, std::string("TrajectoryPlannerROS/local_plan"));

        //ros::Subscriber image_sub = n.subscribe(image_input, 1, image_callback);
        //ros::Subscriber global_costmap_sub = n.subscribe(global_costmap_input, 1, global_costmap_callback);
        //ros::Subscriber local_costmap_sub = n.subscribe(local_costmap_input, 1, local_costmap_callback);
        goal_sub = n.subscribe(goal_input, 1, &state_service::goal_callback, this);
        pose_sub = n.subscribe(pose_input, 1, &state_service::pose_callback, this);
        global_path_sub = n.subscribe(global_path_input, 1, &state_service::global_path_callback, this);
        local_path_sub = n.subscribe(local_path_input, 1, &state_service::local_path_callback, this);
        counter = 0;

        service = n.advertiseService("save_state_service", &state_service::service_callback, this);
    }

    /*void image_callback(const sensor_msgs::Image::ConstPtr& image_msg)
    {
        last_img = image_msg;
    }

    void global_costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        global_last_map = map_msg;
    }

    void local_costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        local_last_map = map_msg;
    }*/

    void goal_callback(const geometry_msgs::PoseStampedConstPtr& goal_msg)
    {
        last_goal = goal_msg;
    }

    void pose_callback(const geometry_msgs::PoseConstPtr& pose_msg)
    {
        last_pose = pose_msg;
    }

    void global_path_callback(const nav_msgs::Path::ConstPtr& path_msg)
    {
        global_last_path = path_msg;
    }

    void local_path_callback(const nav_msgs::Path::ConstPtr& path_msg)
    {
        local_last_path = path_msg;
    }

    void color_around_point(cv::Mat& map, int x, int y, int c)
    {
        for (int i = x - 2; i < x + 3; ++i) {
            for (int j = y - 2; j < y + 3; ++j) {
                // add out-of-bounds check
                if (i < 0 || i >= map.cols || j < 0 || j >= map.rows) {
                    continue;
                }
                map.at<cv::Vec3b>(j, i)[0] = 0;
                map.at<cv::Vec3b>(j, i)[1] = 0;
                map.at<cv::Vec3b>(j, i)[2] = 0;
                map.at<cv::Vec3b>(j, i)[c] = 255;
            }
        }

    }

    void save_map(sensor_msgs::Image& image_msg, const std::string& map_file, const std::string& rgb_map_file,
                  nav_msgs::OccupancyGrid::ConstPtr& last_map, nav_msgs::Path::ConstPtr& last_path, bool save_to_disk)
    {
        // save map
        double map_height = last_map->info.height;
        double map_width = last_map->info.width;
        double map_origin_x = last_map->info.origin.position.x;
        double map_origin_y = last_map->info.origin.position.y;
        double map_res = last_map->info.resolution;
        //mod_num = (int)(map_res/0.05 + 0.5); // solve generically! (0.05 = master_map.resolution...)
        ROS_INFO_STREAM("Map width x height: " << map_width << " x " << map_height);
        // just copy dynamicMap for tableMap and dynMap
        costmap_2d::Costmap2D map(map_width, map_height, map_res, map_origin_x, map_origin_y);

        cv::Mat image = cv::Mat::zeros(map_height, map_width, CV_8UC3);

        // incoming dynamic map to Costmap2D
        for(int i = 0; i < map_width; i++) {
            for(int j = 0; j < map_height; j++) {
                map.setCost(i, j, last_map->data[map.getIndex(i, j)]);
                image.at<cv::Vec3b>(j, i)[0] = last_map->data[map.getIndex(i, j)];
                image.at<cv::Vec3b>(j, i)[1] = last_map->data[map.getIndex(i, j)];
                image.at<cv::Vec3b>(j, i)[2] = last_map->data[map.getIndex(i, j)];
            }
        }

        double x, y;
        int map_x, map_y;

        if (last_path) {
            for (size_t i = 0; i < last_path->poses.size(); ++i) {
                x = last_path->poses[i].pose.position.x;
                y = last_path->poses[i].pose.position.y;
                map.worldToMapEnforceBounds(x, y, map_x, map_y);
                color_around_point(image, map_x, map_y, 1);
            }
        }

        if (last_goal) {
            x = last_goal->pose.position.x;
            y = last_goal->pose.position.y;
            map.worldToMapEnforceBounds(x, y, map_x, map_y);
            color_around_point(image, map_x, map_y, 2);
        }

        if (last_pose) {
            x = last_pose->position.x;
            y = last_pose->position.y;
            map.worldToMapEnforceBounds(x, y, map_x, map_y);
            color_around_point(image, map_x, map_y, 0);
        }

        if (save_to_disk) {
            map.saveMap(map_file);
            // use lossless png compression
            std::vector<int> compression;
            compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression.push_back(0); // no compression
            cv::imwrite(rgb_map_file, image, compression);
        }

        std_msgs::Header header;
        header.seq = counter;
        header.stamp = ros::Time::now();
        header.frame_id = "/map";

        cv_bridge::CvImage global_costmap_tmp = cv_bridge::CvImage(header, "8UC3", image);
        global_costmap_tmp.toImageMsg(image_msg);
    }

    bool service_callback(movebase_state_service::MovebaseStateService::Request& req,
                          movebase_state_service::MovebaseStateService::Response& res)
    {
        sensor_msgs::Image::ConstPtr last_img = ros::topic::waitForMessage<sensor_msgs::Image>(image_input, n, ros::Duration(5));
        if (!last_img) {
            return false;
        }

        if (req.save_to_disk) {
            // save image
            boost::shared_ptr<sensor_msgs::Image> rgb_tracked_object;
            cv_bridge::CvImageConstPtr rgb_cv_img_boost_ptr;
            try {
                rgb_cv_img_boost_ptr = cv_bridge::toCvShare(*last_img, rgb_tracked_object);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return false;
            }
            // use lossless png compression
            std::vector<int> compression;
            compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression.push_back(0); // no compression

            //char buffer[250];
            //sprintf(buffer, "%s/image%06d.png", snapshot_folder.c_str(), counter);
            std::string image_file = snapshot_folder + "/image" + std::to_string(counter) + ".png";
            cv::imwrite(image_file, rgb_cv_img_boost_ptr->image, compression);
        }
        res.camera_image = *last_img;

        nav_msgs::OccupancyGrid::ConstPtr global_costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(global_costmap_input, n, ros::Duration(5));
        if (!global_costmap_msg) {
            return false;
        }
        std::string global_map_file = snapshot_folder + "/global_costmap" + std::to_string(counter) + ".pgm";
        std::string global_rgb_map_file = snapshot_folder + "/global_costmap_path" + std::to_string(counter) + ".png";
        //save_map(global_map_file, global_rgb_map_file, global_last_map, global_last_path);
        save_map(res.global_costmap_image, global_map_file, global_rgb_map_file, global_costmap_msg, global_last_path, req.save_to_disk);

        nav_msgs::OccupancyGrid::ConstPtr local_costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(local_costmap_input, n, ros::Duration(5));
        if (!local_costmap_msg) {
            return false;
        }
        std::string local_map_file = snapshot_folder + "/local_costmap" + std::to_string(counter) + ".pgm";
        std::string local_rgb_map_file = snapshot_folder + "/local_costmap_path" + std::to_string(counter) + ".png";
        //save_map(local_map_file, local_rgb_map_file, local_last_map, local_last_path);
        save_map(res.local_costmap_image, local_map_file, local_rgb_map_file, local_costmap_msg, local_last_path, req.save_to_disk);

        /*res.folder = snapshot_folder;
        res.image_file = image_file;
        res.global_costmap_file = global_rgb_map_file;
        res.local_costmap_file = local_rgb_map_file;*/

        ++counter;

        return true;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "movebase_state_service");

    state_service();
    
    ros::spin();
    
    return 0;
}
