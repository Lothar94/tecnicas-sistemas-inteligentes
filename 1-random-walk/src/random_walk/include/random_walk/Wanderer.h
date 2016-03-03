#ifndef WANDERER_H_
#define WANDERER_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Wanderer
{
public:
	// Tunable parameters
	const static double FORWARD_SPEED_MPS = 2;
	const static double ROTATION_SPEED = 1;
	const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
	const static float MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max

	Wanderer();
	void startMoving();

private:
	ros::NodeHandle node;
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic

	void moveForward();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void rotate(double time);
};

#endif /* Wanderer_H_ */
