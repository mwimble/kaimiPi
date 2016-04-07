#include <ros/ros.h>
#include <unistd.h>
#include "KaimiImu.h"
#include "FetchPrecachedSample.h"

using namespace std;

FetchPrecachedSample::FetchPrecachedSample() {
	cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	currentStrategyPub = nh.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy = strategyHasntStarted;
	publishCurrentStragety(strategyHasntStarted);
}

FetchPrecachedSample& FetchPrecachedSample::Singleton() {
	static FetchPrecachedSample singleton_;
	return singleton_;
}

string FetchPrecachedSample::name() {
	return string("FetchPrecachedSample");
}

void FetchPrecachedSample::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != lastReportedStrategy) {
		lastReportedStrategy = strategy;
		currentStrategyPub.publish(msg);
	}
}

KaimiStrategyFn::RESULT_T FetchPrecachedSample::tick(StrategyContext* strategyContext) {
	static const int DESIRED_Y_FROM_BOTTOM = 15;
	static const int DESIRED_Y_TOLERANCE = 5;
	static const int DESIRED_X_TOLERANCE = 5;

	RESULT_T result = FATAL;

	strategyContext->precachedSampleIsVisibleNearField = KaimiNearField::Singleton().found() && (KaimiNearField::Singleton().x() != 0) && (KaimiNearField::Singleton().y() != 0);
	strategyContext->precachedSampleIsVisibleMidField = !strategyContext->movingViaMidfieldCamera && KaimiNearField::Singleton().found() && (KaimiNearField::Singleton().x() != 0) && (KaimiNearField::Singleton().y() != 0);

	if (strategyContext->needToTurn180) {
		publishCurrentStragety(strategyTurning180);
		if (abs(KaimiImu::Singleton().yaw()) < 3.0) { //#####
			ROS_INFO("[FetchPrecachedSample::tick] Executing 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.5;
			cmdVelPub.publish(cmdVel);
			result = RUNNING;
		} else {
			strategyContext->needToTurn180 = false;
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.0;
			cmdVelPub.publish(cmdVel);
			result = SUCCESS;
			ROS_INFO("[FetchPrecachedSample::tick] COMPLETION of 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
		}

		return result;
	}

	if (strategyContext->precachedSampleFetched) {
		publishCurrentStragety(strategySuccess);
		ROS_INFO("[FetchPrecachedSample::tick] precachedSampleFetched, SUCCESS");
		result = SUCCESS;
	} else if (strategyContext->atPrecachedSample) {
		// Need to  pick up sample
		ROS_INFO_STREAM("[FetchPrecachedSample::tick] atPrecachedSample");
		publishCurrentStragety(strategyPickingUpSample);
		//##### TODO Need to pick up sample.
		strategyContext->needToTurn180;
		result = SUCCESS;
	} else if (strategyContext->precachedSampleIsVisibleNearField) {
		// Move towards sample using nearfield camera.
		publishCurrentStragety(strategyMovingTowardsSampleViaNearfieldCamera);

		double zVel = 0.0;
		double xVel = 0.0;
		int xCenter = KaimiNearField::Singleton().cols() / 2;

		if (abs(KaimiNearField::Singleton().x() - xCenter) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			if (KaimiNearField::Singleton().x() > xCenter) {
				// Need to rotate right.
				zVel = -0.1 - ((KaimiNearField::Singleton().x() - xCenter) * (0.2 / xCenter));
			} else {
				// Need to rotate left.
				zVel = 0.1 + ((xCenter - KaimiNearField::Singleton().x()) * (0.2 / xCenter));
			}
		}

		xVel = (0.5 / KaimiNearField::Singleton().rows()) * (KaimiNearField::Singleton().rows() - KaimiNearField::Singleton().y());
		if (xVel < 0.15) xVel = 0.15;
		cmdVel.linear.x = xVel;
		cmdVel.angular.z = zVel;
		cmdVelPub.publish(cmdVel);

		int xDelta = abs(KaimiNearField::Singleton().x() - xCenter);
		int desiredY = KaimiNearField::Singleton().rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(KaimiNearField::Singleton().y() - desiredY);

		strategyContext->atPrecachedSample = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		ROS_INFO_STREAM("[FetchPrecachedSample::tick] NearField Need to move towards sample, x:"
				<< KaimiNearField::Singleton().x()
				<< ", y: "
				<< KaimiNearField::Singleton().y()
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
		// Move towards sample using midfield camera.
		publishCurrentStragety(strategyMovingTowardsSampleViaMidfieldCamera);
		strategyContext->movingViaMidfieldCamera = true;

		double zVel = 0.0;
		double xVel = 0.0;
		int xCenter = KaimiNearField::Singleton().cols() / 2;

		if (abs(KaimiNearField::Singleton().x() - xCenter) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			if (KaimiNearField::Singleton().x() > xCenter) {
				// Need to rotate right.
				zVel = -0.1 - ((KaimiNearField::Singleton().x() - xCenter) * (0.2 / xCenter));
			} else {
				// Need to rotate left.
				zVel = 0.1 + ((xCenter - KaimiNearField::Singleton().x()) * (0.2 / xCenter));
			}
		}

		xVel = (0.5 / KaimiNearField::Singleton().rows()) * (KaimiNearField::Singleton().rows() - KaimiNearField::Singleton().y());
		if (xVel < 0.15) xVel = 0.15;
		strategyContext->cmdVel.linear.x = xVel;
		strategyContext->cmdVel.angular.z = zVel;
		gettimeofday(&strategyContext->periodStart, NULL);


		int xDelta = abs(KaimiNearField::Singleton().x() - xCenter);
		int desiredY = KaimiNearField::Singleton().rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(KaimiNearField::Singleton().y() - desiredY);

		//strategyContext->atPrecachedSample = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		ROS_INFO_STREAM("[FetchPrecachedSample::tick] MidField Need to move towards sample, x:"
				<< KaimiNearField::Singleton().x()
				<< ", y: "
				<< KaimiNearField::Singleton().y()
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
	} else if (strategyContext->movingViaMidfieldCamera) {
		// Move for 1 second towards sample using the midfield camera.
		publishCurrentStragety(strategyMovingTowardsSampleViaMidfieldCamera);
		cmdVelPub.publish(strategyContext->cmdVel);

		int xCenter = KaimiNearField::Singleton().cols() / 2;
		int xDelta = abs(KaimiNearField::Singleton().x() - xCenter);
		int desiredY = KaimiNearField::Singleton().rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(KaimiNearField::Singleton().y() - desiredY);
		bool shouldHaveBeenSeenByNearField = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		if (shouldHaveBeenSeenByNearField) {
			ROS_WARN("[FetchPrecachedSample::tick] MIDFIELD MOVEMENT SHOULD HAVE CAUSE NEARFIELD TO SEE SAMPLE BY NOW");
			strategyContext->movingViaMidfieldCamera = false;
			result = FAILED;
		} else {
			// Keep this up for one second
			struct timeval now;
			gettimeofday(&now, NULL);
			long seconds = now.tv_sec - strategyContext->periodStart.tv_sec;
			long useconds = now.tv_usec - strategyContext->periodStart.tv_usec;
			double totalElapsedTime = (seconds * 1.0) + (useconds / 1000000.0);
			if (totalElapsedTime > 1.0) {
				strategyContext->movingViaMidfieldCamera = false;
				result = SUCCESS;
			} else {
				// Keep going for one second.
				// ##### TODO Avoid obstacles.
				result = RUNNING;
			}
		}
	} else {
		// Precached sample is not visible.
		publishCurrentStragety(strategyNoSamplSeen);

		// ##### TODO Move forward blindly.
		// ##### TODO If was previously visible, try to find it again.

		ROS_INFO_STREAM("[FetchPrecachedSample::tick] precached sample is not visible");
		result = FAILED; // TODO Finish strategy.
	}

	return result;
}

const string FetchPrecachedSample::strategyHasntStarted = "FetchPrecachedSample: Strategy hasn't started";
const string FetchPrecachedSample::strategyMovingTowardsSampleViaMidfieldCamera = "FetchPrecachedSample: Moving towards sample via midfield camera";
const string FetchPrecachedSample::strategyMovingTowardsSampleViaNearfieldCamera = "FetchPrecachedSample: Moving towards sample via nearfield camera";
const string FetchPrecachedSample::strategyNoSamplSeen = "FetchPrecachedSample:: No target seen";
const string FetchPrecachedSample::strategyPickingUpSample = "FetchPrecachedSample: Picking up sample";
const string FetchPrecachedSample::strategySuccess = "FetchPrecachedSample: SUCCESS";
const string FetchPrecachedSample::strategyTurning180 = "FetchPrecachedSample: Turnning 180 degrees";

