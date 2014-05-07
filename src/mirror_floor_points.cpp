#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <tf/transform_listener.h>

ros::Publisher obstacle_pub;
ros::Publisher floor_pub;

float height;
Eigen::Vector3f normal;
pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud;

float below_threshold;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
	
	Eigen::Vector3f p = -height*normal; // a point in the floor plane
	float d = -p.dot(normal); // = height, d in the plane equation
	
	obstacle_cloud->points.clear();
	obstacle_cloud->points.reserve(cloud->size());
	floor_cloud->points.clear();
	Eigen::Vector3f temp;
	float floor_dist;
	pcl::PointXYZ point;
	for (size_t i = 0; i < cloud->size(); ++i) {
	    
	    /*temp = cloud->points[i].getVector3fMap(); // DEBUG!
	    
	    if (i%640 < 300 && i%640 > 200 && i < 640*200 && i > 640*100) {
	        temp -= 0.06*normal;
	    }*/

        // check signed distance to floor
	    floor_dist = normal.dot(cloud->points[i].getVector3fMap()) + d;
	    //floor_dist = normal.dot(temp) + d; // DEBUG
	    
	    // if enough below, consider a stair point
	    if (floor_dist < below_threshold) {
	        temp = cloud->points[i].getVector3fMap(); // RELEASE
	        point.getVector3fMap() = -(d/normal.dot(temp))*temp + normal*0.11;
	        floor_cloud->points.push_back(point);
	    }
	    else { // add as a normal obstacle or clearing point
	        obstacle_cloud->points.push_back(cloud->points[i]);
	    }
	}

	sensor_msgs::PointCloud2 floor_msg;
    pcl::toROSMsg(*floor_cloud, floor_msg);
	floor_msg.header.frame_id = msg->header.frame_id;

	floor_pub.publish(floor_msg);
	
	sensor_msgs::PointCloud2 obstacle_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
	obstacle_msg.header.frame_id = msg->header.frame_id;
	
	obstacle_pub.publish(obstacle_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subsample_cloud");
	ros::NodeHandle n;
	
	floor_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	obstacle_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	
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
    
    double bt;
    pn.param<double>("below_threshold", bt, 0.05);
    below_threshold = -bt;
    
	ros::Subscriber sub = n.subscribe(input, 1, callback);
    obstacle_pub = n.advertise<sensor_msgs::PointCloud2>(obstacle_output, 1);
    floor_pub = n.advertise<sensor_msgs::PointCloud2>(floor_output, 1);
    
    std::string base_frame("base_link");
    tf::TransformListener listener;
    geometry_msgs::PointStamped pout;
    geometry_msgs::PointStamped pin;
    pin.header.frame_id = camera_frame;
    pin.point.x = 0; pin.point.y = 0; pin.point.z = 0;
    geometry_msgs::Vector3Stamped vout;
    geometry_msgs::Vector3Stamped vin;
    vin.header.frame_id = base_frame;
    vin.vector.x = 0; vin.vector.y = 0; vin.vector.z = 1;
    
    ros::Rate rate(20); // we can afford to update often since pointclouds rare
    while (n.ok()) {
        try {
            listener.transformPoint(base_frame, ros::Time(0), pin, camera_frame, pout);
            height = pout.point.z;
            listener.transformVector(camera_frame, ros::Time(0), vin, base_frame, vout);
            normal = Eigen::Vector3f(vout.vector.x, vout.vector.y, vout.vector.z);
            ros::spinOnce();
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        rate.sleep();
    }
	
	return 0;
}
