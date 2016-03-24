#include <ros/ros.h>
#include "FetchPrecachedSample.h"

FetchPrecachedSample::FetchPrecachedSample() {

}

FetchPrecachedSample& FetchPrecachedSample::Singleton() {
	static FetchPrecachedSample singleton_;
	return singleton_;
}

string FetchPrecachedSample::name() {
	return string("FetchPrecachedSample");
}

KaimiStrategyFn::RESULT_T FetchPrecachedSample::tick(StrategyContext* strategyContext) {
	RESULT_T result = FATAL;
	if (strategyContext->precachedSampleFetched) {
		result = SUCCESS;
	} else if (strategyContext->atPrecachedSample) {
		// Need to  pick up sample
		ROS_INFO_STREAM("[FetchPrecachedSample] atPrecachedSample");
		result = FATAL; // TODO Finish strategy.
	} else if (strategyContext->precachedSampleIsVisible) {
		// Move towards sample.
		ROS_INFO_STREAM("[FetchPrecachedSample] need to move towards sample");
		result = FATAL; // TODO Finish strategy.
	} else {
		// Precached sample is not visible.
		if (strategyContext->precachedSampleIsVeryNear) {
			// We were very near, now we've lost sight. Ready to pick up sample.
			ROS_INFO_STREAM("[FetchPrecachedSample] ready for pickup");
			result = RUNNING;
		}

		ROS_INFO_STREAM("[FetchPrecachedSample] precached sample is not visible");
		result = FATAL; // TODO Finish strategy.
	}

	ROS_INFO_STREAM("[FetchPrecachedSample.tick] result: " << resultToString(result));
	return result;
}

