#ifndef __FETCH_PRECACHED_SAMPLE
#define __FETCH_PRECACHED_SAMPLE

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include "KaimiMidField.h"
#include "KaimiNearField.h"
#include "KaimiStrategyFn.h"

class FetchPrecachedSample : public KaimiStrategyFn {
private:
	// For logging current strategy.
	static const string strategyHasntStarted;
	static const string strategyMovingTowardsSampleViaMidfieldCamera;
	static const string strategyMovingTowardsSampleViaNearfieldCamera;
	static const string strategyNoSamplSeen;
	static const string strategyPickingUpSample;
	static const string strategySuccess;
	static const string strategyTurning180;

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

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

	// Singleton pattern.
	FetchPrecachedSample();
	FetchPrecachedSample(FetchPrecachedSample const&) {};
	FetchPrecachedSample& operator=(FetchPrecachedSample const&) {}

public:
	RESULT_T tick(StrategyContext* strategyContext);

	string name();

	static FetchPrecachedSample& Singleton();
};

#endif
