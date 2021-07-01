/*=============================================================================
#  Author:           Nicolas Queiroga - https://github.com/NicolasQueiroga/
#  Email:            n.macielqueiroga@gmail.com
#  FileName:         main.cpp
#  Version:          1.0.0
=============================================================================*/


#include "Auxiliar/aux.h"
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

/* GLOBAL VALUES */
double posX, posY, radsZ, linVel, angVel;
double dist;
/*###############*/

void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	radsZ = msg->pose.pose.orientation.z;
	linVel = msg->twist.twist.linear.x;
	angVel = msg->twist.twist.angular.z;
}

void laserCB(const sensor_msgs::LaserScan &laser) 
{ 
	dist = laser.ranges[0]; 
}

void imageCB(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	Mat bgr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		resize(cv_ptr->image, bgr, Size(), 0.5, 0.5);
		imshow("img", bgr);
		waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "main");
	ros::NodeHandle nh;


	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/camera/image", 4, imageCB);
	ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCB);
	ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCB);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 3);

	geometry_msgs::Twist msg;
	msg.angular.z = 0.1;

	ros::Rate loop_rate(10);
	while (nh.ok())
	{
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	cout << "\nAn exception has occoured!\n";

	return 0;
}