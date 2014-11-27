#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub; // publishes laser scan with removed edges
float cutoff_angle; // how many angle to cut off in the laser

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int n_points = int(cutoff_angle / msg->angle_increment);
    float angle_min = msg->angle_min + float(n_points)*msg->angle_increment;
    float angle_max = msg->angle_max - float(n_points)*msg->angle_increment;
    int n_last = msg->ranges.size() - n_points;
    
    sensor_msgs::LaserScan msg_out;
    msg_out.ranges.insert(msg_out.ranges.begin(), msg->ranges.begin() + n_points, msg->ranges.begin() + n_last);
    msg_out.intensities.insert(msg_out.intensities.begin(), msg->intensities.begin() + n_points, msg->intensities.begin() + n_last);

    msg_out.angle_min = angle_min;
    msg_out.angle_max = angle_max;
    msg_out.angle_increment = msg->angle_increment;
    msg_out.time_increment = msg->time_increment;
    msg_out.scan_time = msg->scan_time;
    msg_out.range_min = msg->range_min;
    msg_out.range_max = msg->range_max;
    msg_out.header = msg->header;
    pub.publish(msg_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_edges_laser");
    ros::NodeHandle n;

    ros::NodeHandle pn("~");
    // topic of input laser scan
    if (!pn.hasParam("input")) {
        ROS_ERROR("Could not find parameter input.");
        return -1;
    }
    std::string input;
    pn.getParam("input", input);
    
    // topic of output laser scan
    if (!pn.hasParam("output")) {
        ROS_ERROR("Could not find parameter output.");
        return -1;
    }
    std::string output;
    pn.getParam("output", output);

    // how wide angle to cut off at the edges
	if (!pn.hasParam("cutoff_angle")) {
        ROS_ERROR("Could not find parameter cutoff_angle.");
        return -1;
    }
    double temp;
    pn.getParam("cutoff_angle", temp);
    cutoff_angle = M_PI/180.0*temp;
    
    ros::Subscriber sub = n.subscribe(input, 1, callback);
    pub = n.advertise<sensor_msgs::LaserScan>(output, 1);
    
    ros::spin();
	
    return 0;
}
