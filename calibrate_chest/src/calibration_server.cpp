#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "mongodb_store/SetParam.h"

#include <actionlib/server/simple_action_server.h>
#include <calibrate_chest/CalibrateCamera.h>

class CalibrateCameraServer {

    bool do_calibrate; // register and unregister instead
    ros::NodeHandle n;
    actionlib::SimpleActionServer<calibrate_chest::CalibrateCameraAction> server;
    std::string action_name;
    ros::ServiceClient client;
    ros::Subscriber sub;
    std::string camera_topic;
    calibrate_chest::CalibrateCameraFeedback feedback;
    calibrate_chest::CalibrateCameraResult result;

    CalibrateCameraServer(const std::string& name, const std::string& camera_name) :
        do_calibrate(false),
        server(nh, name, boost::bind(&CalibrateCameraServer::execute_cb, this, _1), false),
        action_name(name),
        client(n.serviceClient<mongodb_store::SetParam>("/config_manager/set_param")),
        camera_topic(camera_name + "/depth/points")
    {
        sub = n.subscribe(camera_topic, 1, &CalibrateCameraServer::msg_callback, this);
        server.start();
    }

    bool is_inlier(const Eigen::Vector3f& point, const Eigen::Vector4f plane, double threshold) const
    {
        return fabs(point.dot(plane.segment<3>(0)) + plane(3)) < threshold;
    }

    void plot_best_plane(const pcl::PointCloud<pcl::PointXYZ>& points, const Eigen::Vector4f plane, double threshold) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        for (int i = 0; i < points.size(); ++i) {
            if (is_inlier(points[i].getVector3fMap(), plane, threshold)) {
                inlier_cloud->push_back(points[i]);
            }
            else {
                outlier_cloud->push_back(points[i]);
            }
        }
        
        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.setBackgroundColor(0, 0, 0);
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlier_color_handler(inlier_cloud, 255, 0, 0);
        viewer.addPointCloud(inlier_cloud, inlier_color_handler, "inliers");
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outlier_color_handler(outlier_cloud, 0, 0, 255);
        viewer.addPointCloud(outlier_cloud, outlier_color_handler, "outliers");
        
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
        
    }

    void compute_plane(Eigen::Vector4f& plane, const pcl::PointCloud<pcl::PointXYZ>& points, int* inds) const
    {
        Eigen::Vector3f first = points[inds[1]].getVector3fMap() - points[inds[0]].getVector3fMap();
        Eigen::Vector3f second = points[inds[2]].getVector3fMap() - points[inds[0]].getVector3fMap();
        Eigen::Vector3f normal = first.cross(second);
        normal.normalize();
        plane.segment<3>(0) = normal;
        plane(3) = -normal.dot(points[inds[0]].getVector3fMap());
    }

    void extract_height_and_angle(const Eigen::Vector4f& plane)
    {
        feedback.status = "Checking and saving calibration...";
        server.publishFeedback(feedback);

        ROS_INFO("Ground plane: %f, %f, %f, %f", plane(0), plane(1), plane(2), plane(3));
        double dist = fabs(plane(3)/plane(2)); // distance along z axis
        double height = fabs(plane(3)/plane.segment<3>(0).squaredNorm()); // height
        ROS_INFO("Distance to plane along camera axis: %f", dist);
        ROS_INFO("Height above ground: %f", height);
        double angle = asin(height/dist);
        double angle_deg = 180.0f*angle/M_PI;
        ROS_INFO("Angle radians: %f", angle);
        ROS_INFO("Angle degrees: %f", angle_deg);

        result.angle = angle_deg;
        result.height = height;

        if (fabs(45.0f - angle_deg) > 3.0f) {
            result.status = "Angle not close enough to 45 degrees.";
            server.setAborted(result);
            return;
        }
        
        mongodb_store::SetParam srv;
        char buffer[250];
        
        // store height above ground in datacentre
        ros::param::set("/chest_xtion_height", height);
        sprintf(buffer, "{\"path\":\"/chest_xtion_height\",\"value\":%f}", height);
        srv.request.param = buffer;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call set height, is config manager running?");
        }
        
        // store angle between camera and horizontal plane
        ros::param::set("/chest_xtion_angle", angle);
        sprintf(buffer, "{\"path\":\"/chest_xtion_angle\",\"value\":%f}", angle);
        srv.request.param = buffer;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call set angle, is config manager running?");
        }

        result.status = "Successfully computed and saved calibration.";
        server.setSucceeded(result);
    }

    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (!do_calibrate) {
            return;
        }
        do_calibrate = false;

        feedback.status = "Calibrating...";
        feedback.progress = 0.0f;
        server.publishFeedback(feedback);

        ROS_INFO("Got a pointcloud, calibrating...");
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        
        int nbr = cloud.size();
        
        int max = 1000; // ransac iterations
        double threshold = 0.02; // threshold for plane inliers
        
        Eigen::Vector4f best_plane; // best plane parameters found
        int best_inliers = -1; // best number of inliers
        
        int inds[3];
        
        Eigen::Vector4f plane;
        int inliers;
        for (int i = 0; i < max; ++i) {
            
            for (int j = 0; j < 3; ++j) {
                inds[j] = rand() % nbr; // get a random point
            }
            
            // check that the points aren't the same
            if (inds[0] == inds[1] || inds[0] == inds[2] || inds[1] == inds[2]) {
                continue;
            }
            
            compute_plane(plane, cloud, inds);
            inliers = 0;
            for (int j = 0; j < nbr; j += 30) { // count number of inliers
                if (is_inlier(cloud[j].getVector3fMap(), plane, threshold)) {
                    ++inliers;
                }
            }
            
            if (inliers > best_inliers) {
                best_plane = plane;
                best_inliers = inliers;
                feedback.progress = float(i+1)/float(max);
                server.publishFeedback(feedback);
            }
        }
        
        extract_height_and_angle(best_plane); // find parameters and feed them to datacentre
        //plot_best_plane(cloud, best_plane, threshold); // visually evaluate plane fit
    }

    void execute_cb(const calibrate_chest::CalibrateCameraGoalConstPtr& goal)
    {
        if (goal->command == "calibrate") {
            do_calibrate = true;
        }
        else {
            result.status = "Enter command \"calibrate\" or \"publish\".";
            server.setAborted(result);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibrate_chest");
    CalibrateCameraServer calibrate(ros::this_node::getName(), "chest_xtion");
    ros::spin();
	
	return 0;
}
