#include <ros/ros.h>
#include <ros/console.h>

#include "KaimiNearField.h"

KaimiNearField* kaimiNearField = NULL;

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinect_strategy_node");
	kaimiNearField = KaimiNearField::Singleton();
	ros::spin();
	return 0;
}

