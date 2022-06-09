#include "robot/actions.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "main");
	ros::NodeHandle nh;
	Actions robot = Actions(&nh);

	ros::Rate loop(10);
	while (nh.ok())
	{
		// robot.makeSquare();
		robot.followRoad();

		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}