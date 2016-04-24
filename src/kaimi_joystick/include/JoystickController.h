#ifndef __JOYSTICK_CONTROLLER
#define __JOYSTICK_CONTROLLER

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <string>

using namespace std;

class JoystickController {
private:
	ros::NodeHandle nh_;		// Ros node handle.
	ros::Publisher cmdVelPub_;
	ros::Subscriber joystickSubscriber_;
	
	int enableButton_;			// Must be pressed to enable joystick control.
	int turboButton_;			// If pressed, boost velocities.
	string joyTopicName_;		// Topic for listening to joystick messages.
	int cmdVelMessageRate_;		// Rate to issue cmd_vel messages;
	double xScale_;				// For scaling joystick for linear.x;
	double zScale_;				// For scaling joystick for angular.z;
	double turboScale_;			// For scaling turbo boose;

	//void JoyCb(const sensor_msgs::Joy::ConstPtr& joy);

	JoystickController();
	JoystickController(JoystickController const&) {};
	JoystickController& operator=(JoystickController const&) {};
	static JoystickController* singleton;
	

public:
	int getTurboButton() { return turboButton_; }
	int getEnableButton() { return enableButton_; }
	double getXScale() { return xScale_; }
	double getZScale() { return zScale_; }
	double getTurboScale() { return turboScale_; }
	ros::Publisher getPublisher() { return cmdVelPub_; }

	static JoystickController& Singleton();
};

#endif