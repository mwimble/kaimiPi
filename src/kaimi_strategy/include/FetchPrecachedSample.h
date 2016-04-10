#ifndef __FETCH_PRECACHED_SAMPLE
#define __FETCH_PRECACHED_SAMPLE

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include "KaimiMidField.h"
#include "KaimiNearField.h"
#include "KaimiStrategyFn.h"
#include "StrategyContext.h"

class FetchPrecachedSample : public KaimiStrategyFn {
private:
	// For logging current strategy.
	static const string strategyHasntStarted;
	static const string strategyMovingTowardsSampleViaMidfieldCamera;
	static const string strategyMovingTowardsSampleViaNearfieldCamera;
	static const string strategyNoSampleSeen;
	static const string strategyPickingUpSample;
	static const string strategySuccess;
	static const string strategyTurning180;
	static const string strategyWaitingForPauseOff;
	static const string strategyWaitingForPauseOn;
	static const string stragetyWaitingOnArm;

	static const int GRIPPER_READ_PIN;		// Using Broadcom pin number 17
	static const int GRIPPER_WRITE_PIN;		// Using Broadcom pin number 18

	static StrategyContext& strategyContext;

	int gripperReadHandle;	// Used to see if gripper is done.
	int gripperWriteHandle;	// Used to tell gripper to do its thing.

	// For sending robot movement commands.
	geometry_msgs::Twist cmdVel;

	// Topic to publish robot movements.
	ros::Publisher cmdVelPub;

	// Topic to publish current strategy.
	ros::Publisher currentStrategyPub;

	// ROS node handle.
	ros::NodeHandle nh;

	// Receives basePaused messages.
	ros::Subscriber pausedSub;

	// Callback for basePaused message
	void pausedCallback(const std_msgs::String& commandMessage);

	// Keeps track of whether robot is paused.
	bool isPaused;


	// To help log strategy only when it changes.
	string lastReportedStrategy;

	// Used to prevent a slew of "No target seen" messages;
	bool reportedNoFound;

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

	// Singleton pattern.
	FetchPrecachedSample();
	FetchPrecachedSample(FetchPrecachedSample const&) {};
	FetchPrecachedSample& operator=(FetchPrecachedSample const&) {}

public:
	RESULT_T tick();

	string name();

	static FetchPrecachedSample& Singleton();
};

#endif
