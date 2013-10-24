#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

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

	float width = 3.0;
	float height = 1.2;
	int width_res = 60;
	int height_res = 24;
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.resize(width_res*height_res);

	float min_x = -0.5*width;
	float min_y = 0.0;
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
		msg_cloud.header.stamp = ros::Time::now();
		pub.publish(msg_cloud);
		rate.sleep();
	}
	
	return 0;
}
