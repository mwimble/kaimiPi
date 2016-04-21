// TODO dynamic params for everything.

#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <unistd.h>

#include "FindObject.h"

FindObject* findObject; 

int main(int argc, char** argv) {
	fprintf(stderr, "main start, about to ros::init");
	ros::init(argc, argv, "kinect_mid_camera_publisher_node");
	fprintf(stderr, "About to create FindOejct");
	findObject = &FindObject::Singleton();
	fprintf(stderr, "About to spin");
	ros::spin();
	return 0;
}

