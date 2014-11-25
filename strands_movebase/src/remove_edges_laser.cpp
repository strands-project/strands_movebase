#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub; // publishes pointcloud with removed edges
double cutoff_angle; // how many angle to cut off in the laser

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan msg_out = *msg;
    msg_out.angle_min = msg->angle_min + cutoff_angle;
    msg_out.angle_max = msg->angle_max - cutoff_angle;
	msg_out.header = msg->header;
	pub.publish(msg_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_edges_laser");
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
    if (!pn.hasParam("output")) {
        ROS_ERROR("Could not find parameter output.");
        return -1;
    }
    std::string output;
    pn.getParam("output", output);

    // how many pixels to cut off in the depth image
	if (!pn.hasParam("cutoff_angle")) {
        ROS_ERROR("Could not find parameter cutoff_angle.");
        return -1;
    }
    pn.getParam("cutoff_angle", cutoff_angle);
    
	ros::Subscriber sub = n.subscribe(input, 1, callback);
    pub = n.advertise<sensor_msgs::LaserScan>(output, 1);
    
    ros::spin();
	
	return 0;
}
