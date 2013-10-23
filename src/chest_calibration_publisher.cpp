#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "chest_calibration_publisher");
	ros::NodeHandle n;
    
    double height, angle;
    n.param<double>("/chest_xtion_height", height, 1.10f);
    n.param<double>("/chest_xtion_angle", angle, 0.72f);
    
    std::cout << height << std::endl;
    std::cout << angle << std::endl;
    ros::Rate rate(1.0f);
    
    while (n.ok()) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.12f, 0.0f, height));
        transform.setRotation(tf::createQuaternionFromRPY(0.0f, angle, 0.0f));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/chest_xtion_link"));
        rate.sleep();
    }
    
    return 0;
}
