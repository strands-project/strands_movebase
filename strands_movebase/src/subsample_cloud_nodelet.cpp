#include "strands_movebase/subsample_cloud_nodelet.h"

// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include "strands_movebase/noise_approximate_voxel_grid.h"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(strands_movebase::subsample_cloud_nodelet, nodelet::Nodelet)

namespace strands_movebase {

void subsample_cloud_nodelet::onInit()
{
    NODELET_INFO("Initializing subsample_cloud nodelet...");

    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pn = getPrivateNodeHandle();

    // topic of input cloud
    if (!pn.hasParam("input")) {
        ROS_ERROR("Could not find parameter input.");
    }
    std::string input;
    pn.getParam("input", input);
    NODELET_INFO("With input %s", input.c_str());

    // topic of output cloud
    if (!pn.hasParam("output")) {
        ROS_ERROR("Could not find parameter output.");
    }
    std::string output;
    pn.getParam("output", output);
    NODELET_INFO("And output %s", output.c_str());

    pn.param<double>("resolution", resolution, 0.05);
    pn.param<int>("min_points", min_points, 5);
    pn.param<int>("skip_points", skip_points, 20);

    time = ros::Time::now();

    sub = nh.subscribe(input, 1, &subsample_cloud_nodelet::callback, this);
    pub = nh.advertise<sensor_msgs::PointCloud2>(output, 1);

    /*ros::Rate rate(5); // updating at 5 hz, slightly faster than move_base
    while (n.ok()) {
        rate.sleep();
        ros::spinOnce();
    }*/
}

void subsample_cloud_nodelet::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ros::Time new_time = ros::Time::now();
    if ((new_time - time).toSec() < 0.2) {
        return;
    }
    time = new_time;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ> voxel_cloud;

    noise_approximate_voxel_grid sor(min_points, skip_points); // a bit faster than nvg, not as accurate
    sor.setInputCloud(cloud);
    sor.setLeafSize(resolution, resolution, resolution);
    sor.filter(voxel_cloud);

    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(voxel_cloud, msg_cloud);
    msg_cloud.header = msg->header;

    pub.publish(msg_cloud);
}

} // movebase_processing
