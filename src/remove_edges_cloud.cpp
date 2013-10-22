#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;
float cutoff;

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

    // topic of the depth and rgb images
    if (!n.hasParam("/remove_edges_cloud/camera")) {
        ROS_ERROR("Could not find parameter camera.");
        return -1;
    }
    std::string camera_topic;
    n.getParam("/remove_edges_cloud/camera", camera_topic);

	if (!n.hasParam("/remove_edges_cloud/cutoff")) {
        ROS_ERROR("Could not find parameter cutoff.");
        return -1;
    }
    n.getParam("/remove_edges_cloud/cutoff", cutoff);
    
	ros::Subscriber sub = n.subscribe(camera_topic + "/depth/points_subsampled", 1, callback);
    pub = n.advertise<sensor_msgs::PointCloud2>(camera_topic + "/depth/points_clearing", 1);
    
    ros::spin();
	
	return 0;
}
