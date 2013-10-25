#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

/* This node creates a flat point cloud ahead
of the robot to clear points above a certain
height, this should not be needed in a
working system. */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "synthesize_wall_cloud");
	ros::NodeHandle n;
    
    // topic of output cloud
    if (!n.hasParam("/synthesize_wall_cloud/output")) {
        ROS_ERROR("Could not find parameter output.");
        return -1;
    }
    std::string output;
    n.getParam("/synthesize_wall_cloud/output", output);

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(output, 1);

	float width = 3.0; // width of point wall
	float height = 1.2; // height of point wall
	int width_res = 60; // number of points along width
	int height_res = 24; // number of points along height
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.resize(width_res*height_res);

	float min_x = -0.5*width;
	float min_y = 0.0; // start at ground plane
	float step_x = width/float(width_res);
	float step_y = height/float(height_res);
	
	for (int x = 0; x < width_res; ++x) {
		for (int y = 0; y < height_res; ++y) {
			int ind = x*height_res + y;
			cloud.points[ind].x = 2.5;
			cloud.points[ind].y = min_x + x*step_x;
			cloud.points[ind].z = min_y + y*step_y;
		}
	}

	sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(cloud, msg_cloud);
	msg_cloud.header.frame_id = "/base_link";
    
    ros::Rate rate(10);
	
	while (n.ok()) {
	    // just change the timestamp before publishing,
	    // everything else the same
		msg_cloud.header.stamp = ros::Time::now();
		pub.publish(msg_cloud);
		rate.sleep();
	}
	
	return 0;
}
