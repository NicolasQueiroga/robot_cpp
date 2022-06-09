#ifndef __ROBOT_HPP_
#define __ROBOT_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "aux_opencv.hpp"

class Robot
{
protected:
    image_transport::Subscriber img_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;
    ros::Publisher pub;
    ros::Publisher pub_pose;
    geometry_msgs::Pose2D pose2d;
    geometry_msgs::Twist msg;
    cv::Point2d pos;
    float laser_dist;
    cv::Point pmin;
    cv::Point pcenter;
    bool gotCenter;
    double angularCoef;

public:
    Robot(ros::NodeHandle *nh);
    ~Robot();

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
    void laserCB(const sensor_msgs::LaserScan &laser);
    void imageCB(const sensor_msgs::ImageConstPtr &msg);

    cv::Mat imgProc(cv::Mat bgr);
    void getPosition(cv::Point2d *pos_ptr, double *angle_ptr);
    void getDistance(float *dist_ptr);
    void setSpeed(double v = 0, double w = 0);
};

#endif