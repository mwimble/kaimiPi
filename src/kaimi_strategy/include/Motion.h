#ifndef __MOTION
#define __MOTION

#include <ros/ros.h>
#include <ros/console.h>

class Motion {
private:
	static const int MID_FIELD_DESIRED_Y_FROM_BOTTOM = 35;
	static const int NEAR_FIELD_DESIRED_Y_FROM_BOTTOM = 35;

	// Topic to publish robot movements.
	ros::Publisher cmdVelPub;

	// ROS node handle.
	ros::NodeHandle nh;

	// Singleton pattern.
	Motion();
	Motion(Motion const&) {};
	Motion& operator=(Motion const&) {}

public:
	static const int MID_FIELD_DESIRED_X = 640;
	static const int MID_FIELD_DESIRED_X_TOLERANCE = 15;

	static const int MID_FIELD_DESIRED_Y = 900;
	static const int MID_FIELD_DESIRED_Y_TOLERANCE = 20;

	static const int NEAR_FIELD_DESIRED_X = 440;
	static const int NEAR_FIELD_DESIRED_X_TOLERANCE = 7;

	static const int NEAR_FIELD_DESIRED_Y = 455;
	static const int NEAR_FIELD_DESIRED_Y_TOLERANCE = 5;

	void seekViaMidfield(float& xDelta, float& yDelta);

	void seekViaNearfield(float& xDelta, float& yDelta);

	static Motion& Singleton();
};

#endif