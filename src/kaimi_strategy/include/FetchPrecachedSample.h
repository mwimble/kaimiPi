#ifndef __FETCH_PRECACHED_SAMPLE
#define __FETCH_PRECACHED_SAMPLE

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>


#include "KaimiNearField.h"
#include "KaimiStrategyFn.h"

class FetchPrecachedSample : public KaimiStrategyFn {
private:
	geometry_msgs::Twist cmdVel;

	ros::Publisher cmdVelPub;

	KaimiNearField* kaimiNearField;

	ros::NodeHandle nh;

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
