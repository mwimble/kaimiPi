#include "JoystickController.h"
#include <geometry_msgs/Twist.h>

void JoyCb(const sensor_msgs::Joy::ConstPtr& joy) {
	geometry_msgs::Twist cmdVel;
	JoystickController& jc = JoystickController::Singleton();
	bool joystickEnabled = joy->buttons[jc.getEnableButton()] == 1;
	bool turboEnabled = joy->buttons[jc.getTurboButton()] == 1;

	//ROS_INFO("[JoyCb] enable_button: %d, turboEnabled: %d", joystickEnabled, turboEnabled);
	if (joystickEnabled) {
		cmdVel.linear.x = joy->axes[4] * jc.getXScale() * (turboEnabled ? jc.getTurboScale() : 1.0);
		cmdVel.angular.z = joy->axes[3] * jc.getZScale() * (turboEnabled ? jc.getTurboScale() : 1.0);
		jc.getPublisher().publish(cmdVel);
	}
}

JoystickController::JoystickController() {
	ros::param::get("~joystick_topic_name", joyTopicName_);
	ros::param::get("~cmd_vel_message_rate", cmdVelMessageRate_);
	ros::param::get("~enable_button", enableButton_);
	ros::param::get("~boost_button", turboButton_);
	ros::param::get("~x_scale", xScale_);
	ros::param::get("~z_scale", zScale_);
	ros::param::get("~turbo_scale", turboScale_);
	ROS_INFO("PARAM joystick_topic_name: %s", joyTopicName_.c_str());
	ROS_INFO("PARAM cmd_vel_message_rate: %d", cmdVelMessageRate_);
	ROS_INFO("PARAM enable_button: %d", enableButton_);
	ROS_INFO("PARAM boost_button: %d", turboButton_);
	ROS_INFO("PARAM x_scale: %f", xScale_);
	ROS_INFO("PARAM z_scale: %f", zScale_);
	ROS_INFO("PARAM turbo_scale: %f", turboScale_);

	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 4);
	joystickSubscriber_ = nh_.subscribe<sensor_msgs::Joy>(joyTopicName_, 5, &JoyCb);
}

JoystickController& JoystickController::Singleton() {
	static JoystickController singleton_;
	return singleton_;
}