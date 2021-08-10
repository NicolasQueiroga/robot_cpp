/*=============================================================================
#  Author:           Nicolas Queiroga - https://github.com/NicolasQueiroga/
#  Email:            n.macielqueiroga@gmail.com
#  FileName:         main.cpp
#  Version:          1.0.0
=============================================================================*/

#include "Auxiliar/aux.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;
using namespace cv;

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

public:
	Robot(ros::NodeHandle *nh)
	{
		image_transport::ImageTransport it(*nh);
		img_sub = it.subscribe("/camera/image", 4, &Robot::imageCB, this);
		odom_sub = nh->subscribe("/odom", 10, &Robot::odomCB, this);
		laser_sub = nh->subscribe("/scan", 10, &Robot::laserCB, this);
		pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 3);
		pos = cv::Point(0, 0);
		laser_dist = 0.0;
	}
	~Robot() {}

	void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
	{
		pose2d.x = msg->pose.pose.position.x;
		pose2d.y = msg->pose.pose.position.y;
		tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		if (yaw * 180 / CV_PI < 0)
			yaw += 2 * CV_PI;
		pose2d.theta = yaw * 180 / CV_PI;

		// std::cout << "(" << pose2d.x << ", " << pose2d.y << ") and theta = " << pose2d.theta << std::endl;
	}

	void laserCB(const sensor_msgs::LaserScan &laser)
	{
		laser_dist = laser.ranges[0];
	}

	void imageCB(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv::Mat bgr, processed;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv::resize(cv_ptr->image, bgr, cv::Size(), 0.5, 0.5);
			processed = imgProc(bgr);
			cv::imshow("img", processed);
			cv::waitKey(1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}

	cv::Mat imgProc(cv::Mat bgr)
	{
		return bgr;
	}

	void getPosition(cv::Point2d *pos_ptr, double *angle_ptr)
	{
		*pos_ptr = cv::Point2d(pose2d.x, pose2d.y);
		*angle_ptr = pose2d.theta;
	}

	void getDistance(float *dist_ptr)
	{
		*dist_ptr = laser_dist;
	}

	void setSpeed(double v = 0, double w = 0)
	{
		msg.linear.x = v;
		msg.angular.z = w;
		pub.publish(msg);
	}
};

class Actions : public Robot
{
private:
	bool turn;
	bool walk;
	int angles[3];
	int i;
	cv::Point2d p1;
	cv::Point2d p;
	double rad;
	float dist;

public:
	Actions(ros::NodeHandle *nh) : Robot(nh)
	{
		// Infos
		p = {0.0, 0.0};
		rad = 0.0;
		dist = 0.0;

		// makeSquare
		turn = false;
		walk = true;
		angles[0] = 90;
		angles[1] = 180;
		angles[2] = 270;
		i = 0;
		p1 = {0.0, 0.0};
	}

	void makeSquare()
	{
		getDistance(&dist);
		getPosition(&p, &rad);
		double d = pow((pow((p.x - p1.x), 2) + pow((p.y - p1.y), 2)), 0.5);

		if (walk && !turn)
		{
			std::cout << "debug: walk\n";
			if (d > 0.8)
			{
				p1 = p;
				setSpeed();
				walk = false;
				if (i == 3)
					turn = false;
				else
					turn = true;
			}
			else
			{
				setSpeed(0.3, 0);
			}
		}
		else if (!walk && turn)
		{
			if (rad >= angles[i] && rad < angles[i] + 5)
			{
				std::cout << "\n\n\ndebug: turn ENDED\n\n\n";
				setSpeed();
				turn = false;
				walk = true;
				i++;
			}
			else
			{
				std::cout << "debug: turn\n";
				setSpeed(0, 0.3);
			}
		}
		else
		{
			if (rad >= 0 && rad < 3)
			{
				i = 0;
				walk = true;
			}
			else
			{
				setSpeed(0, 0.3);
			}
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "main");
	ros::NodeHandle nh;
	Actions robot = Actions(&nh);

	ros::Rate loop(10);
	while (nh.ok())
	{
		robot.makeSquare();
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}