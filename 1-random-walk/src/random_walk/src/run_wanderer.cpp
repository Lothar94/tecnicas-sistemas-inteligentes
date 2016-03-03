#include "Wanderer.h"

int main(int argc, char **argv) {
	// Initiate new ROS node named "stopper"
	ros::init(argc, argv, "wanderer");

	// Create new stopper object
	Wanderer wanderer;

	// Start the movement
	wanderer.startMoving();

	return 0;
};



