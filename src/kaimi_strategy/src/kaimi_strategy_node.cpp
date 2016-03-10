#include <ros/ros.h>
#include <ros/console.h>

#include "KaimiNearField.h"

KaimiNearField* kaimiNearField;

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinect_strategy_node");
	kaimiNearField = new KaimiNearField();
	ros::spin();
	return 0;
}

