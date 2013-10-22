#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub; // publishes pointcloud with removed edges
double cutoff; // how many pixels to cut off in the depth image

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
	
	Eigen::Matrix3f K; // camera matrix of short range primesense
	K << 570.34f, 0.0f, 314.5f, 0.0f, 570.34f, 235.5f, 0.0f, 0.0f, 1.0f;
	
	pcl::PointCloud<pcl::PointXYZ> new_cloud;
	new_cloud.resize(cloud.size());
	Eigen::Vector3f p;
	int counter = 0;
	for (int i = 0; i < cloud.size(); ++i) {
	    // transform points to image plane and make sure they are within bounds
		p = K*cloud.points[i].getVector3fMap();
		p = p / p(2); // we don't have any points at z = 0
		if (p(0) > 0.0f + cutoff && p(0) < 629.0f - cutoff &&
			p(1) > 0.0f + cutoff && p(1) < 471.0f - cutoff) {
			new_cloud.points[counter] = cloud.points[i];
			++counter;
		}
	}
	new_cloud.resize(counter);

	sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(new_cloud, msg_cloud);
	msg_cloud.header.frame_id = msg->header.frame_id;
	pub.publish(msg_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_edges_cloud");
	ros::NodeHandle n;	

    // topic of input cloud
    if (!n.hasParam("/remove_edges_cloud/input")) {
        ROS_ERROR("Could not find parameter input.");
        return -1;
    }
    std::string input;
    n.getParam("/remove_edges_cloud/input", input);
    
    // topic of output cloud
    if (!n.hasParam("/remove_edges_cloud/output")) {
        ROS_ERROR("Could not find parameter output.");
        return -1;
    }
    std::string output;
    n.getParam("/remove_edges_cloud/output", output);

    // how many pixels to cut off in the depth image
	if (!n.hasParam("/remove_edges_cloud/cutoff")) {
        ROS_ERROR("Could not find parameter cutoff.");
        return -1;
    }
    n.getParam("/remove_edges_cloud/cutoff", cutoff);
    
	ros::Subscriber sub = n.subscribe(input, 1, callback);
    pub = n.advertise<sensor_msgs::PointCloud2>(output, 1);
    
    ros::spin();
	
	return 0;
}
