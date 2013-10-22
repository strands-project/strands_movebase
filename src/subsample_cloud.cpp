#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include "noise_voxel_grid.h"

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
	pcl::PointCloud<pcl::PointXYZ> voxel_cloud;
	
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
    noise_voxel_grid sor(5, 20);
	sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
	sor.filter(voxel_cloud);

	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(new_cloud);
	sor.setMeanK(20);
	sor.setStddevMulThresh(1.0);
	sor.filter(voxel_cloud);*/

	sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(voxel_cloud, msg_cloud);
	msg_cloud.header.frame_id = msg->header.frame_id;

	pub.publish(msg_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subsample_cloud");
	ros::NodeHandle n;
	
	/*
    // topic of the depth and rgb images
    if (!n.hasParam("/image_player_node/camera_topic")) {
        ROS_ERROR("Could not find parameter camera_topic.");
        return -1;
    }
    std::string camera_topic;
    n.getParam("/image_player_node/camera_topic", camera_topic);
    */
    std::string camera_topic = "chest_xtion";
    
	//ros::Subscriber sub = n.subscribe(camera_topic + "/depth/points", 1, callback);
	ros::Subscriber sub = n.subscribe("/points", 1, callback);
    pub = n.advertise<sensor_msgs::PointCloud2>(camera_topic + "/depth/points_subsampled", 1);
    
    ros::spin();
	
	return 0;
}
