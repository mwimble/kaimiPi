#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include "KaimiNearField.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinect_strategy_node");
	ros::NodeHandle nh;
	ros::Publisher cmdVelPub;
	geometry_msgs::Twist cmdVel;
	KaimiNearField& kaimiNearField = KaimiNearField::Singleton();
	ros::Rate rate(10); // 10Hz loop rate

	// If near field not found, did we previously see it as we were
	// advancing very near to it?
	bool wasAdvancingOnVeryNear = false;

	cmdVelPub = nh.advertise<std_msgs::String>("cmd_vel", 1);
		
	while (ros::ok()) {
		ros::spinOnce();

		cmdVel.linear.x = 0;
		cmdVel.linear.y = 0;
		cmdVel.linear.z = 0;
		cmdVel.angular.x = 0;
		cmdVel.angular.y = 0;
		cmdVel.angular.z = 0;
		// Handle sample foundd in near field camera.
		if (kaimiNearField.found()) {
			switch (kaimiNearField.leftRight()) {
				case KaimiNearField::FAR_LEFT:
					// Need to rotate left a lot.
					wasAdvancingOnVeryNear = false;
					cmdVel.angular.z = 1.0;
					cmdVelPub.publish(cmdVel);
					ROS_INFO("Moving fast left to center sample");
					continue;

				case KaimiNearField::LEFT:
					// Need to rotate left a bit.
					wasAdvancingOnVeryNear = false;
					cmdVel.angular.z = 0.5;
					cmdVelPub.publish(cmdVel);
					ROS_INFO("Moving left to center sample");
					continue;

				case KaimiNearField::CENTER:
					// Sample is ahead.
					switch (kaimiNearField.farNear()) {
						case KaimiNearField::VERY_FAR_AWAY:
							// Need to move ahead very fast.
							wasAdvancingOnVeryNear = false;
							cmdVel.linear.x = 2;
							cmdVelPub.publish(cmdVel);
							ROS_INFO("Moving fast forward to sample");
							continue;

						case KaimiNearField::FAR_AWAY:
							// Need to move ahead normal.
							wasAdvancingOnVeryNear = false;
							cmdVel.linear.x = 1;
							cmdVelPub.publish(cmdVel);
							ROS_INFO("Moving forward to sample");
							continue;

						case KaimiNearField::NEAR:
							// Need to move ahead slowly.
							wasAdvancingOnVeryNear = false;
							cmdVel.linear.x = 0.5;
							cmdVelPub.publish(cmdVel);
							ROS_INFO("Moving slowly forward to sample");
							continue;

						case KaimiNearField::VERY_NEAR:
							// Need to move ahead very slowly.
							wasAdvancingOnVeryNear = true;
							cmdVel.linear.x = 0.4;
							cmdVelPub.publish(cmdVel);
							wasAdvancingOnVeryNear = true;
							ROS_INFO("Moving very slowly forward to sample");
							continue;
					}

					continue;

				case KaimiNearField::RIGHT:
					// Need to rotate right a bit.
					wasAdvancingOnVeryNear = false;
					cmdVel.angular.z = -0.5;
					cmdVelPub.publish(cmdVel);
					ROS_INFO("Moving right to center sample");
					continue;

				case KaimiNearField::FAR_RIGHT:
					// Need to rotate right a lot.
					wasAdvancingOnVeryNear = false;
					cmdVel.angular.z = -1.0;
					cmdVelPub.publish(cmdVel);
					ROS_INFO("Moving fast right to center sample");
					continue;
			}
		} else {
			// No found.
			if (wasAdvancingOnVeryNear) {
				ROS_INFO("IN POSITION to pick up sample");
			}
		}

		rate.sleep();
	}

	return 0;
}

