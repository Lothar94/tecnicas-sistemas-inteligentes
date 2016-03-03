#include <random>
#include <sstream>
#include "std_msgs/String.h"
#include "Wanderer.h"
#include "geometry_msgs/Twist.h"


const double Wanderer::FORWARD_SPEED_MPS = 2;
const double Wanderer::ROTATION_SPEED = 2;
const double Wanderer::MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
const double Wanderer::MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
const float Wanderer::MIN_PROXIMITY_RANGE_M = 0.5;

Wanderer::Wanderer()
{
	// Advertise a new publisher for the simulated robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("base_scan", 1, &Wanderer::scanCallback, this);
}

// Send a velocity command
void Wanderer::moveForward() {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
};

// Process the incoming laser scan message
void Wanderer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE_M) {
		ROS_INFO("Rotate!");
		std::default_random_engine generator;
		std::uniform_real_distribution<double> distribution(1,5);
		auto rotation_time = std::bind ( distribution, generator );

		std::uniform_int_distribution<int> distr_sign(0, 2);
		auto sign = std::bind ( distr_sign, generator );
		//int sign = (distr_sign(generator) - 1) * 2 - 1;

		std_msgs::String msg;
		std::stringstream converter;
		converter << sign() << " - " << rotation_time();
		msg.data = converter.str();
		ROS_INFO("%s", msg.data.c_str());

		rotate(sign(), rotation_time());
	}
}

void Wanderer::rotate(int direction, double time){
	ros::Rate rate(1/time);
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.angular.z = direction * ROTATION_SPEED;
	commandPub.publish(msg);
	rate.sleep();
	msg.angular.x = 0;
	commandPub.publish(msg);
}

void Wanderer::startMoving()
{
	ros::Rate rate(10);
	ROS_INFO("Start moving");

	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok()) {
		moveForward();
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}
}
