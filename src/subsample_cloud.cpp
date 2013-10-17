#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    
	int skip = 50;
    int nbr = cloud.size();
	int new_nbr = nbr / skip;
	pcl::PointCloud<pcl::PointXYZ> new_cloud;
	new_cloud.resize(new_nbr);
    
	int counter = 0;
	for (int i = 0; i < nbr; i += skip) {
		new_cloud[counter] = cloud[i];
		++counter;
	}

	sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(new_cloud, msg_cloud);
	msg_cloud.header.frame_id = "/chest_xtion_depth_optical_frame";

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
	pub = n.advertise<sensor_msgs::PointCloud2>("/points_subsampled", 1);
    
    ros::spin();
	
	return 0;
}
