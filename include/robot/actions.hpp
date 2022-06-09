#ifndef __ROBOT_HPP_
#define __ROBOT_HPP_

#include "robot/robot.hpp"

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
    int limit;

public:
    Actions(ros::NodeHandle *nh);
    
	void makeSquare(void);
    void followRoad(void);
};

#endif