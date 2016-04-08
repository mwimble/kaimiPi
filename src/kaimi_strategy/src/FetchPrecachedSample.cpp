#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include "KaimiImu.h"
#include "KaimiMidField.h"
#include "FetchPrecachedSample.h"

using namespace std;

FetchPrecachedSample::FetchPrecachedSample() {
	pausedSub = nh.subscribe("basePaused", 1, &FetchPrecachedSample::pausedCallback, this);
	cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	currentStrategyPub = nh.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy = strategyHasntStarted;
	publishCurrentStragety(strategyHasntStarted);
}

FetchPrecachedSample& FetchPrecachedSample::Singleton() {
	static FetchPrecachedSample singleton_;
	return singleton_;
}

void FetchPrecachedSample::pausedCallback(const std_msgs::String& msg) {
	if (msg.data == "paused") {
		isPaused = true;
	} else {
		isPaused = false;
	}

	ROS_INFO("[FetchPrecachedSample::pausedCallback] paused: %d", isPaused);
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

// ##### TODO If was approaching via near-field and sample disappears, back up a bit.
// ##### TODO If trending towards sample and it moves significantly, don't track new position.
// ##### TODO When sample found, what for pause on/pause off before continuing.

KaimiStrategyFn::RESULT_T FetchPrecachedSample::tick(StrategyContext* strategyContext) {
	static const int DESIRED_Y_FROM_BOTTOM = 35;
	static const int DESIRED_Y_TOLERANCE = 5;
	static const int DESIRED_X_TOLERANCE = 5;

	RESULT_T result = FATAL;

	if (!strategyContext->lookingForPrecachedSample) {
		result = SUCCESS;
		return result;
	}

	strategyContext->precachedSampleIsVisibleNearField = KaimiNearField::Singleton().found();
	strategyContext->precachedSampleIsVisibleMidField = !strategyContext->movingViaMidfieldCamera && KaimiMidField::Singleton().found();

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
			strategyContext->lookingForHome = true;
			result = SUCCESS;
			if (strategyContext->atPrecachedSample) {
				strategyContext->lookingForPrecachedSample = false;
			}

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
		strategyContext->needToTurn180 = true;
		result = SUCCESS;
	} else if (strategyContext->precachedSampleIsVisibleNearField) {
		// Move towards sample using nearfield camera.
		publishCurrentStragety(strategyMovingTowardsSampleViaNearfieldCamera);

		if (strategyContext->minX < 0.11) {
			// Quick fix.
			strategyContext->minX = 0.11;
		}

		double x = KaimiNearField::Singleton().x();
		double y = KaimiNearField::Singleton().y();
		double zVel = 0.0;
		double xVel = 0.0;
		int xCenter = KaimiNearField::Singleton().cols() / 2;

		if (abs(x - xCenter) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			zVel = ((xCenter - x) * (0.2 / xCenter));
		}

		xVel = (0.6 / KaimiNearField::Singleton().rows()) * (KaimiNearField::Singleton().rows() - y);
		
		// ##### TODO Need some sort of averaging here rather than looking for instantaneous response from last command.
		if (abs(strategyContext->lastX - x) < 2) {
			strategyContext->countXStill++;
			// Last Z velocity should have rotated and didn't.
			if (strategyContext->countXStill >= 4) {
				// New Z velocity magnitude is less than or equal to previous, so it's unlikely to cause a change.
				strategyContext->minZ = strategyContext->minZ+ 0.01;
			}
		} else {
			strategyContext->countXStill = 0;
		}

		if (abs(zVel) < strategyContext->minZ) {
			zVel = zVel >= 0 ? strategyContext->minZ : -strategyContext->minZ;
		}

		static const double maxZVel = 0.3;

		if (abs(zVel) > maxZVel) zVel = zVel >= 0.0 ? maxZVel : -maxZVel;


		// ##### TODO Need some sort of averaging here rather than looking for instantaneous response from last command.
		if (abs(strategyContext->lastY - y) < 2) {
			strategyContext->countYStill++;
			ROS_INFO("Y delta<1: %7.2f, lastY: %7.2f, y: %7.2f, xVel: %7.2f, minX: %7.2f, countYStill: %d", abs(strategyContext->lastY - y), strategyContext->lastY, y, xVel, strategyContext->minX, strategyContext->countYStill);
			// Last X velocity should have moved forward or backward and didn't.
			if (strategyContext->countYStill >= 4) {
				// New X velocity magnitude is less than or equal to previous, so it's unlikely to cause a change.
				strategyContext->minX = strategyContext->minX + 0.01;
				ROS_INFO("new minX: %7.4f, new xVel: %7.4f", strategyContext->minX, xVel);
			}
		} else {
			strategyContext->countYStill = 0;
			ROS_INFO("Y delta>=1: %7.2f, lastY: %7.2f, y: %7.2f", abs(strategyContext->lastY - y), strategyContext->lastY, y);
		}

		if (abs(xVel) < strategyContext->minX) {
			ROS_INFO("override xVel %7.4f with minX: %7.4f", xVel,  strategyContext->minX);
			xVel = xVel >= 0 ? strategyContext->minX : -strategyContext->minX;
		}

		static const double maxXVel = 0.50;

		if (abs(xVel) > maxXVel) xVel = xVel >= 0.0 ? maxXVel : -maxXVel;

		strategyContext->lastX = x;
		strategyContext->lastY = y;

		cmdVel.linear.x = xVel;
		cmdVel.angular.z = zVel;
		cmdVelPub.publish(cmdVel);

		int xDelta = abs(x - xCenter);
		int desiredY = KaimiNearField::Singleton().rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(y - desiredY);

		strategyContext->atPrecachedSample = (xDelta < DESIRED_X_TOLERANCE) &&
			((yDelta < DESIRED_Y_TOLERANCE) || (y > desiredY));

		ROS_INFO_STREAM("[FetchPrecachedSample::tick] NearField Need to move towards sample, x:"
				<< x
				<< ", y: "
				<< y
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
				<< DESIRED_Y_TOLERANCE
				<< ", minXVel: "
				<< strategyContext->minX
				<< ", minZVel: "
				<< strategyContext->minZ);


		result = RUNNING; // TODO Finish strategy.
	} else if (0 && /*#####*/ strategyContext->precachedSampleIsVisibleMidField) {
		// Move towards sample using midfield camera.
		publishCurrentStragety(strategyMovingTowardsSampleViaMidfieldCamera);
		strategyContext->movingViaMidfieldCamera = true;

		double x = KaimiMidField::Singleton().x();
		double y = KaimiMidField::Singleton().y();
		double zVel = 0.0;
		double xVel = 0.0;
		int xCenter = KaimiMidField::Singleton().cols() / 2;

		if (abs(x - xCenter) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			zVel = ((xCenter - x) * (0.1 / xCenter));
		}

		xVel = (0.6 / KaimiMidField::Singleton().rows()) * (KaimiMidField::Singleton().rows() - y);
		if (xVel < 0.15) xVel = 0.15;
		strategyContext->cmdVel.linear.x = xVel;
		strategyContext->cmdVel.angular.z = zVel;
		gettimeofday(&strategyContext->periodStart, NULL);


		int xDelta = abs(x - xCenter);
		int desiredY = KaimiMidField::Singleton().rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(y - desiredY);

		//strategyContext->atPrecachedSample = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		ROS_INFO_STREAM("[FetchPrecachedSample::tick] MidField Need to move towards sample, x:"
				<< x
				<< ", y: "
				<< y
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

		double x = KaimiMidField::Singleton().x();
		double y = KaimiMidField::Singleton().y();
		int xCenter = KaimiMidField::Singleton().cols() / 2;
		int xDelta = abs(x - xCenter);
		int desiredY = KaimiMidField::Singleton().rows() - DESIRED_Y_FROM_BOTTOM;
		int yDelta = abs(y - desiredY);
		bool shouldHaveBeenSeenByNearField = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		if (shouldHaveBeenSeenByNearField) {
			ROS_WARN("[FetchPrecachedSample::tick] MIDFIELD MOVEMENT SHOULD HAVE CAUSED NEARFIELD TO SEE SAMPLE BY NOW");
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
		publishCurrentStragety(strategyNoSampleSeen);

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
const string FetchPrecachedSample::strategyNoSampleSeen = "FetchPrecachedSample:: No target seen";
const string FetchPrecachedSample::strategyPickingUpSample = "FetchPrecachedSample: Picking up sample";
const string FetchPrecachedSample::strategySuccess = "FetchPrecachedSample: SUCCESS";
const string FetchPrecachedSample::strategyTurning180 = "FetchPrecachedSample: Turnning 180 degrees";

