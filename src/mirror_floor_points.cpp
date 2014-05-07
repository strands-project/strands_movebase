#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

ros::Publisher pub;

double height;
Eigen::Vector3d normal;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
	
	for (size_t i = 0; i < cloud->size(); ++i) {
	    
	}

	sensor_msgs::PointCloud2 msg_cloud;
    //pcl::toROSMsg(voxel_cloud, msg_cloud);
	msg_cloud.header.frame_id = msg->header.frame_id;

	pub.publish(msg_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subsample_cloud");
	ros::NodeHandle n;
	
	ros::NodeHandle pn("~");
    // topic of input cloud
    if (!pn.hasParam("input")) {
        ROS_ERROR("Could not find parameter input.");
        return -1;
    }
    std::string input;
    pn.getParam("input", input);
    
    // topic of output cloud
    if (!pn.hasParam("obstacle_output")) {
        ROS_ERROR("Could not find parameter obstacle_output.");
        return -1;
    }
    std::string obstacle_output;
    pn.getParam("obstacle_output", obstacle_output);
    
    // topic of output cloud
    if (!pn.hasParam("camera_frame")) {
        ROS_ERROR("Could not find parameter camera_frame.");
        return -1;
    }
    std::string camera_frame;
    pn.getParam("camera_frame", camera_frame);
    
    // topic of output cloud
    if (!pn.hasParam("floor_output")) {
        ROS_ERROR("Could not find parameter floor_output.");
        return -1;
    }
    std::string floor_output;
    pn.getParam("floor_output", floor_output);
    
	ros::Subscriber sub = n.subscribe(input, 1, callback);
    pub = n.advertise<sensor_msgs::PointCloud2>(obstacle_output, 1);
    
    std::string base_frame("map");//"base_link");
    tf::TransformListener listener;
    geometry_msgs::PointStamped pout;
    geometry_msgs::PointStamped pin;
    pin.header.frame_id = base_frame;
    pin.point.x = 0; pin.point.y = 0; pin.point.z = 0;
    geometry_msgs::Vector3Stamped vout;
    geometry_msgs::Vector3Stamped vin;
    vin.header.frame_id = base_frame;
    vin.vector.x = 0; vin.vector.y = 0; vin.vector.z = 1;
    
    ros::Rate rate(5); // updating at 5 hz, slightly faster than move_base
    while (n.ok()) {
        tf::StampedTransform transform;
        try {
            //listener.lookupTransform(camera_frame, "base_link", ros::Time(0), transform);
            listener.transformPoint(camera_frame, ros::Time(0), pin, base_frame, pout);
            height = pout.point.z;
            listener.transformVector(camera_frame, ros::Time(0), vin, base_frame, vout);
            normal = Eigen::Vector3d(vout.vector.x, vout.vector.y, vout.vector.z);
            //std::cout << transform << std::endl;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        std::cout << height << std::endl;
        std::cout << normal.transpose() << std::endl;
        rate.sleep();
        ros::spinOnce();
    }
	
	return 0;
}
