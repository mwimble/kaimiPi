#include <ros/ros.h>
#include "FetchPrecachedSample.h"

FetchPrecachedSample::FetchPrecachedSample() {
	cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	kaimiNearField = &KaimiNearField::Singleton();
	kaimiMidField = &KaimiMidField::Singleton();
}

FetchPrecachedSample& FetchPrecachedSample::Singleton() {
	static FetchPrecachedSample singleton_;
	return singleton_;
}

string FetchPrecachedSample::name() {
	return string("FetchPrecachedSample");
}

KaimiStrategyFn::RESULT_T FetchPrecachedSample::tick(StrategyContext* strategyContext) {
	static const int DESIRED_Y_FROM_BOTTOM = 15;
	static const int DESIRED_Y_TOLERANCE = 5;
	static const int DESIRED_X_TOLERANCE = 5;

	RESULT_T result = FATAL;

	strategyContext->precachedSampleIsVisibleNearField = kaimiNearField->found() && (kaimiNearField->x() != 0) && (kaimiNearField->y() != 0);
	strategyContext->precachedSampleIsVisibleMidField = kaimiMidField->found() && (kaimiMidField->x() != 0) && (kaimiMidField->y() != 0);

	if (strategyContext->precachedSampleFetched) {
		ROS_INFO("[FetchPrecachedSample] precachedSampleFetched, SUCCESS");
		result = SUCCESS;
	} else if (strategyContext->atPrecachedSample) {
		// Need to  pick up sample
		ROS_INFO_STREAM("[FetchPrecachedSample] atPrecachedSample");
		result = FATAL; // TODO Finish strategy.
	} else if (strategyContext->precachedSampleIsVisibleNearField) {
		// Move towards sample.
		double zVel = 0.0;
		double xVel = 0.0;
		int xCenter = kaimiNearField->cols() / 2;

		if (abs(kaimiNearField->x() - xCenter) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			if (kaimiNearField->x() > xCenter) {
				// Need to rotate right.
				zVel = -0.1 - ((kaimiNearField->x() - xCenter) * (0.2 / xCenter));
			} else {
				// Need to rotate left.
				zVel = 0.1 + ((xCenter - kaimiNearField->x()) * (0.2 / xCenter));
			}
		}

		xVel = (0.5 / kaimiNearField->rows()) * (kaimiNearField->rows() - kaimiNearField->y());
		if (xVel < 0.15) xVel = 0.15;
		cmdVel.linear.x = xVel;
		cmdVel.angular.z = zVel;
		cmdVelPub.publish(cmdVel);

		int xDelta = abs(kaimiNearField->x() - xCenter);
		int desiredY = kaimiNearField->rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(kaimiNearField->y() - desiredY);

		strategyContext->atPrecachedSample = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		ROS_INFO_STREAM("[kaimi_stragety_node] NearField Need to move towards sample, x:"
				<< kaimiNearField->x()
				<< ", y: "
				<< kaimiNearField->y()
				<< ", xVel: "
				<< xVel
				<< ", zVel: "
				<< zVel
				<< ", desired x: "
				<< xCenter
				<< ", xDelta ("
				<< xDelta
				<< ") needs to be under "
				<< DESIRED_X_TOLERANCE
				<< ", desired y: "
				<< desiredY
				<< ", yDelta ("
				<< yDelta
				<< ") needs to be under "
				<< DESIRED_Y_TOLERANCE);


		result = RUNNING; // TODO Finish strategy.
	} else if (strategyContext->precachedSampleIsVisibleMidField) {
		// Move towards sample.
		double zVel = 0.0;
		double xVel = 0.0;
		int xCenter = kaimiMidField->cols() / 2;

		if (abs(kaimiMidField->x() - xCenter) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			if (kaimiMidField->x() > xCenter) {
				// Need to rotate right.
				zVel = -0.1 - ((kaimiMidField->x() - xCenter) * (0.2 / xCenter));
			} else {
				// Need to rotate left.
				zVel = 0.1 + ((xCenter - kaimiMidField->x()) * (0.2 / xCenter));
			}
		}

		xVel = (0.5 / kaimiMidField->rows()) * (kaimiMidField->rows() - kaimiMidField->y());
		if (xVel < 0.15) xVel = 0.15;
		cmdVel.linear.x = xVel;
		cmdVel.angular.z = zVel;
		cmdVelPub.publish(cmdVel);

		int xDelta = abs(kaimiMidField->x() - xCenter);
		int desiredY = kaimiMidField->rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(kaimiMidField->y() - desiredY);

		strategyContext->atPrecachedSample = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		ROS_INFO_STREAM("[kaimi_stragety_node] MidField Need to move towards sample, x:"
				<< kaimiMidField->x()
				<< ", y: "
				<< kaimiMidField->y()
				<< ", xVel: "
				<< xVel
				<< ", zVel: "
				<< zVel
				<< ", desired x: "
				<< xCenter
				<< ", xDelta ("
				<< xDelta
				<< ") needs to be under "
				<< DESIRED_X_TOLERANCE
				<< ", desired y: "
				<< desiredY
				<< ", yDelta ("
				<< yDelta
				<< ") needs to be under "
				<< DESIRED_Y_TOLERANCE);


		result = RUNNING; // TODO Finish strategy.
	} else {
		// Precached sample is not visible.

		// TODO If was previously visible, try to find it again.

		ROS_INFO_STREAM("[FetchPrecachedSample] precached sample is not visible");
		result = FAILED; // TODO Finish strategy.
	}

	return result;
}

