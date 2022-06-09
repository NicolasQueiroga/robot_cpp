#include "robot/robot.hpp"

Robot::Robot(ros::NodeHandle *nh)
{
    image_transport::ImageTransport it(*nh);
    img_sub = it.subscribe("/camera/image", 4, &Robot::imageCB, this);
    odom_sub = nh->subscribe("/odom", 10, &Robot::odomCB, this);
    laser_sub = nh->subscribe("/scan", 10, &Robot::laserCB, this);
    pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 3);
    pos = cv::Point2d(0, 0);
    laser_dist = 0.0;
    pmin = cv::Point(0, 0);
    pcenter = cv::Point(0, 0);
    gotCenter = false;
    angularCoef = 0.0;
}

Robot::~Robot() {}

void Robot::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
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

void Robot::laserCB(const sensor_msgs::LaserScan &laser)
{
    laser_dist = laser.ranges[0];
}

void Robot::imageCB(const sensor_msgs::ImageConstPtr &msg)
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

cv::Mat Robot::imgProc(cv::Mat bgr)
{
    if (!gotCenter)
    {
        pcenter = cv::Point(bgr.size().width / 2, bgr.size().height / 2);
        gotCenter = true;
    }
    angularCoef = linearRegression(bgr, &pmin);
    std::vector<int> ids;
    detectAruco(&bgr, &ids);
    for (int i : ids)
        std::cout << "aruco id " << i << "\n";
    return bgr;
}

void Robot::getPosition(cv::Point2d *pos_ptr, double *angle_ptr)
{
    *pos_ptr = cv::Point2d(pose2d.x, pose2d.y);
    *angle_ptr = pose2d.theta;
}

void Robot::getDistance(float *dist_ptr)
{
    *dist_ptr = laser_dist;
}

void Robot::setSpeed(double v, double w)
{
    msg.linear.x = v;
    msg.angular.z = w;
    pub.publish(msg);
}