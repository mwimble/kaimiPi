#include <ros/ros.h>
#include "JoystickController.h"

JoystickController* joystickController;

int main(int argc, char** argv) {
	ros::init(argc, argv, "kaimi_joystick_node");
	joystickController = &JoystickController::Singleton();
	ros::spin();
	return 0;
}