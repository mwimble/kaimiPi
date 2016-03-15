#include <ros/ros.h>
#include <ros/console.h>
#include "DiffDriveController.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinect_strategy_node");
	DiffDriveController& diffDriveController = DiffDriveController::Singleton();
	ros::spin();
	return 0;
}

