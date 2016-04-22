#include <geometry_msgs/Twist.h>

#include "KaimiMidField.h"
#include "KaimiNearField.h"
#include "Motion.h"

Motion::Motion() {
	cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

Motion& Motion::Singleton() {
	static Motion singleton_;
	return singleton_;
}

void Motion::seekViaMidfield(float& xDelta, float& yDelta) {
	double x = KaimiMidField::Singleton().x();
	double y = KaimiMidField::Singleton().y();
	double zVel = 0.0;
	double xVel = 0.0;

	// Either rotate or move straight -- not both at once.
	float xSlewCorrection = abs(x - MID_FIELD_DESIRED_X) / 6400;
	float desiredXTolerance = MID_FIELD_DESIRED_X_TOLERANCE + ((960 - y) / 20);
	if (abs(x - MID_FIELD_DESIRED_X) > desiredXTolerance) {
		// Need to rotate to center
		if (x > MID_FIELD_DESIRED_X) {
			// Need to rotate clockwise.
			zVel = -0.24 - xSlewCorrection;
		} else {
			// Need to rotate counterclockwise.
			zVel = 0.19 + xSlewCorrection;
		}

		xVel = 0.2;
	} else {
		xVel = 0.4;
		zVel = -0.05;
	}

	geometry_msgs::Twist cmdVel;	
	cmdVel.linear.x = xVel;
	cmdVel.angular.z = zVel;
	cmdVelPub.publish(cmdVel);

	xDelta = abs(x - MID_FIELD_DESIRED_X);
	yDelta = abs(y - MID_FIELD_DESIRED_Y);

	/*
	ROS_INFO_STREAM("[Motion::seekViaMidfield] MIDField Need to move towards sample, x:"
			<< x
			<< ", y: "
			<< y
			<< ", xVel: "
			<< xVel
			<< ", zVel: "
			<< zVel
			<< ", desired x: "
			<< Motion::MID_FIELD_DESIRED_X
			<< ", xDelta ("
			<< xDelta
			<< ") needs to be under "
			<< desiredXTolerance
			<< ", desired y: "
			<< Motion::MID_FIELD_DESIRED_Y
			<< ", yDelta ("
			<< yDelta
			<< ") needs to be under "
			<< Motion::MID_FIELD_DESIRED_Y_TOLERANCE
			<< ", xSlewCorrection: "
			<< xSlewCorrection);
*/
}

void Motion::seekViaNearfield(float& xDelta, float& yDelta) {
	double x = KaimiNearField::Singleton().x();
	double y = KaimiNearField::Singleton().y();
	double zVel = 0.0;
	double xVel = 0.0;

	// Either rotate or move straight -- not both at once.
	float xSlewCorrection = abs(x - MID_FIELD_DESIRED_X) / 3200;
	float desiredXTolerance = NEAR_FIELD_DESIRED_X_TOLERANCE + ((480 - y) / 20);
	if (abs(x - NEAR_FIELD_DESIRED_X) > desiredXTolerance) {
		// Need to rotate to center
		if (x > NEAR_FIELD_DESIRED_X) {
			// Need to rotate clockwise.
			zVel = -0.23 - xSlewCorrection;
		} else {
			// Need to rotate counterclockwise.
			zVel = 0.18 + xSlewCorrection;
		}
	} else {
		if (y < (NEAR_FIELD_DESIRED_Y - 100)) {
			// Move a bit fast.
			xVel = 0.26;
			zVel = -0.05;
		} else if (y < NEAR_FIELD_DESIRED_Y) {
			// Move slowly.
			xVel = 0.21;
			zVel = -0.03;
		}
	}

	geometry_msgs::Twist cmdVel;	
	cmdVel.linear.x = xVel;
	cmdVel.angular.z = zVel;
	cmdVelPub.publish(cmdVel);

	xDelta = abs(x - NEAR_FIELD_DESIRED_X);
	yDelta = abs(y - NEAR_FIELD_DESIRED_Y);
	/*
	ROS_INFO_STREAM("[Motion::seekViaNearfield] NearField Need to move towards sample, x:"
			<< x
			<< ", y: "
			<< y
			<< ", xVel: "
			<< xVel
			<< ", zVel: "
			<< zVel
			<< ", desired x: "
			<< Motion::NEAR_FIELD_DESIRED_X
			<< ", xDelta ("
			<< xDelta
			<< ") needs to be under "
			<< desiredXTolerance
			<< ", desired y: "
			<< Motion::NEAR_FIELD_DESIRED_Y
			<< ", yDelta ("
			<< yDelta
			<< ") needs to be under "
			<< Motion::NEAR_FIELD_DESIRED_Y_TOLERANCE
			<< ", xSlewCorrection: "
			<< xSlewCorrection);
*/
}
