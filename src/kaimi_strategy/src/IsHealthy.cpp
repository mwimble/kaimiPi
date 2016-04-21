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

KaimiStrategyFn::RESULT_T IsHealthy::tick() {
	RESULT_T result = SUCCESS;
	return result;
}

