#include <ros/ros.h>
#include "FetchPrecachedSample.h"

FetchPrecachedSample::FetchPrecachedSample() {
	kaimiNearField = &KaimiNearField::Singleton();
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

	strategyContext->precachedSampleIsVisible = kaimiNearField->found() && (kaimiNearField->x() != 0) && (kaimiNearField->y() != 0);

	if (strategyContext->precachedSampleFetched) {
		result = SUCCESS;
	} else if (strategyContext->atPrecachedSample) {
		// Need to  pick up sample
		ROS_INFO_STREAM("[FetchPrecachedSample] atPrecachedSample");
		result = FATAL; // TODO Finish strategy.
	} else if (strategyContext->precachedSampleIsVisible) {
		// Move towards sample.
		ROS_INFO_STREAM("[FetchPrecachedSample] need to move towards sample");
		result = RUNNING; // TODO Finish strategy.
	} else {
		// Precached sample is not visible.
		if (strategyContext->precachedSampleIsVeryNear) {
			// We were very near, now we've lost sight. Ready to pick up sample.
			ROS_INFO_STREAM("[FetchPrecachedSample] ready for pickup");
			result = RUNNING;
		}

		ROS_INFO_STREAM("[FetchPrecachedSample] precached sample is not visible");
		result = FAILED; // TODO Finish strategy.
	}

	ROS_INFO_STREAM("[FetchPrecachedSample.tick] result: " << resultToString(result));
	return result;
}

