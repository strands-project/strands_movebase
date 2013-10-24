#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "chest_calibration_publisher");
	ros::NodeHandle n;
    
    // get calibration with respect to ground plane from the calibrate_chest node
    double height, angle;
    n.param<double>("/chest_xtion_height", height, 1.10f); // get the height calibration
    n.param<double>("/chest_xtion_angle", angle, 0.72f); // get the angle calibration
    
    ROS_INFO("Setting height of chest camera: %f", height);
    ROS_INFO("Setting angle of chest camera: %f", angle);
    ros::Rate rate(1.0f);
    
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/chest_calibration_publisher/state", 1);
    
    int counter = 0; // only publish for the first 10 secs, transforms will stay in tf
    while (n.ok() && counter < 10) {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.velocity.resize(2);
        joint_state.name[0] = "chest_xtion_height_joint";
        joint_state.name[1] = "chest_xtion_tilt_joint";
        joint_state.position[0] = height;
        joint_state.position[1] = angle;
        joint_state.velocity[0] = 0;
        joint_state.velocity[1] = 0;
        pub.publish(joint_state);
        rate.sleep();
        ++counter;
    }
    
    ROS_INFO("Stopping to publish chest transform after 10 seconds, quitting...");
    
    return 0;
}
