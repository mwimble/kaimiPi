#include <ros/ros.h>
#include "IsHealthy.h"

IsHealthy::IsHealthy() {

}

IsHealthy& IsHealthy::Singleton() {
	static IsHealthy singleton_;
	return singleton_;
}

string IsHealthy::name() {
	return string("IsHealthy");
}

KaimiStrategyFn::RESULT_T IsHealthy::tick(StrategyContext* strategyContext) {
	RESULT_T result = SUCCESS;
	//ROS_INFO_STREAM("[IsHealthy.tick] result: " << resultToString(result));
	return result;
}

