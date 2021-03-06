#ifndef WANDERER_H_
#define WANDERER_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Wanderer
{
public:
	// Tunable parameters
	const static double FORWARD_SPEED_MPS;
	const static double MIN_SCAN_ANGLE_RAD;
	const static double MAX_SCAN_ANGLE_RAD;
	const static float MIN_PROXIMITY_RANGE_M; // Should be smaller than sensor_msgs::LaserScan::range_max

	Wanderer();
	void startMoving();

private:
	ros::NodeHandle node;
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
	int next_rotation_direction;
	float min_distance_to_wall;
	float closestRange;

	void moveForward();
	void turnAround();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void rotate(int sign, double time);
};

#endif /* Wanderer_H_ */
