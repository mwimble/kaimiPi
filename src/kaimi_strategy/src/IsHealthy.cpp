#include <ros/ros.h>
#include "IsHealthy.h"

string IsHealthy::name() {
	return string("IsHealthy");
}

KaimiStrategyFn::RESULT_T IsHealthy::tick() {
	static int c = 0;

	RESULT_T result = ++c > 5 ? FATAL : (c > 3 ? SUCCESS : FAILED);
	ROS_INFO_STREAM("[IsHealthy.tick] result: " << resultToString(result));
	return result;
}

