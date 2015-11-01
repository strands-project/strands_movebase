#ifndef SUBSAMPLE_CLOUD_NODELET_H
#define SUBSAMPLE_CLOUD_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>

namespace strands_movebase {

class subsample_cloud_nodelet : public nodelet::Nodelet {
private:
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Time time;
    double resolution;
    int min_points;
    int skip_points;
public:
    virtual void onInit();
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

} // movebase_processing

#endif // SUBSAMPLE_CLOUD_NODELET_H
