#ifndef __GO_HOME
#define __GO_HOME

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include "KaimiMidField.h"
#include "KaimiNearField.h"
#include "KaimiStrategyFn.h"
#include "StrategyContext.h"

class GoHome : public KaimiStrategyFn {
private:
	// For logging current strategy.
	static const string strategyHasntStarted;
	static const string strategyMovingTowardsHomeViaMidfieldCamera;
	static const string strategyMovingTowardsHomeViaNearfieldCamera;
	static const string strategyNoHomeSeen;
	static const string strategySuccess;
	static const string strategyTurning180;

	static StrategyContext& strategyContext;

	// For sending robot movement commands.
	geometry_msgs::Twist cmdVel;

	// Topic to publish robot movements.
	ros::Publisher cmdVelPub;

	// Topic to publish current strategy.
	ros::Publisher currentStrategyPub;

	// ROS node handle.
	ros::NodeHandle nh;

	// To help log strategy only when it changes.
	string lastReportedStrategy;

	// Used to prevent a slew of "No target seen" messages;
	bool reportedNoFound;

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

	// Singleton pattern.
	GoHome();
	GoHome(GoHome const&) {};
	GoHome& operator=(GoHome const&) {}

public:
	RESULT_T tick();

	string name();

	static GoHome& Singleton();
};

#endif
