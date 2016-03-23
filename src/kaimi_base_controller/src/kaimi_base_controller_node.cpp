#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include "DiffDriveController.h"

void kaimiSignalHandler(int sig) {
	ROS_ERROR("[kaimiSignalHandler] SIGNAL_HANDLER");
	fprintf(stderr, "\n\n[kaimiSignalHandler] SIGNAL_HANDLER\n\n");
	DiffDriveController::Singleton().stop();
}

int main(int argc, char** argv) {

	signal(SIGINT, kaimiSignalHandler);

	ros::init(argc, argv, "kinect_strategy_node");
	DiffDriveController& diffDriveController = DiffDriveController::Singleton();
	while (ros::ok()) {
		if (ros::isShuttingDown()) {
			fprintf(stderr, "\n\n[kaimi_base_controller_node] stopping #1\n\n");
			DiffDriveController::Singleton().stop();
			ROS_ERROR("[kaimi_base_controller_node] stopping #1");
			break;
		}

		ros::spinOnce();
		if (ros::isShuttingDown()) {
			fprintf(stderr, "\n\n[kaimi_base_controller_node] stopping #2\n\n");
			DiffDriveController::Singleton().stop();
			ROS_ERROR("[kaimi_base_controller_node] stopping #2");
			break;
		}
	}

	ROS_INFO("[kaimi_base_controller_node] stopping");
	DiffDriveController::Singleton().stop();
	return 0;
}

