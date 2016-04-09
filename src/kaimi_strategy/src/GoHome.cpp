#include <ros/ros.h>
#include <unistd.h>
#include "KaimiImu.h"
#include "KaimiMidField.h"
#include "GoHome.h"

using namespace std;

GoHome::GoHome() {
	cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	currentStrategyPub = nh.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy = strategyHasntStarted;
	publishCurrentStragety(strategyHasntStarted);
}

GoHome& GoHome::Singleton() {
	static GoHome singleton_;
	return singleton_;
}

string GoHome::name() {
	return string("GoHome");
}

void GoHome::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != lastReportedStrategy) {
		lastReportedStrategy = strategy;
		currentStrategyPub.publish(msg);
	}
}

// ##### TODO If was approaching via near-field and home disappears, back up a bit.
// ##### TODI If trending towards sample and it moves significantly, don't track new position.

KaimiStrategyFn::RESULT_T GoHome::tick(StrategyContext* strategyContext) {
	// x: 449, y: 437
	static const int DESIRED_X = 449;
	static const int DESIRED_Y = 437;
	static const int DESIRED_Y_FROM_BOTTOM = 35;
	static const int DESIRED_Y_TOLERANCE = 5;
	static const int DESIRED_X_TOLERANCE = 5;

	RESULT_T result = FATAL;

	if (!strategyContext->lookingForHome) {
		result = SUCCESS;
		return result;
	}

	strategyContext->homeIsVisibleNearField = KaimiNearField::Singleton().found();
	strategyContext->homeIsVisibleMidField = !strategyContext->movingViaMidfieldCamera && KaimiMidField::Singleton().found();

	if (strategyContext->needToTurn180) {
		publishCurrentStragety(strategyTurning180);
		if (abs(KaimiImu::Singleton().yaw() - strategyContext->startYaw) < 3.0) {
			ROS_INFO("[GoHome::tick] Executing 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.5;
			cmdVelPub.publish(cmdVel);
			result = RUNNING;
		} else {
			strategyContext->needToTurn180 = false;
			publishCurrentStragety("!!! DONE !!!"); //#####
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.0;
			cmdVelPub.publish(cmdVel);
			strategyContext->lookingForHome = false;
			result = SUCCESS;
			ROS_INFO("[GoHome::tick] COMPLETION of 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
		}

		return result;
	}

	if (strategyContext->atHome) {
		// Need to turn around, for show.
		ROS_INFO_STREAM("[GoHome::tick] atHome");
		strategyContext->needToTurn180 = true;
		strategyContext->startYaw = KaimiImu::Singleton().yaw();
		result = SUCCESS;
	} else if (strategyContext->homeIsVisibleNearField) {
		// Move towards home using nearfield camera.
		publishCurrentStragety(strategyMovingTowardsHomeViaNearfieldCamera);

		/*-- Common between GoHome and FetchPrecachedSample --*/
		if (strategyContext->minX < 0.11) {
			// Quick fix.
			strategyContext->minX = 0.11;
		}

		double x = KaimiNearField::Singleton().x();
		double y = KaimiNearField::Singleton().y();
		double zVel = 0.0;
		double xVel = 0.0;

		if (abs(x - DESIRED_X) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			zVel = ((DESIRED_X - x) * (0.6 / DESIRED_X));
		}

		xVel = (0.6 / KaimiNearField::Singleton().rows()) * (KaimiNearField::Singleton().rows() - y);
		
		if (abs(strategyContext->lastX - x) < 2) {
			strategyContext->countXStill++;
			// Last Z velocity should have rotated and didn't.
			if (strategyContext->countXStill >= 4) {
				// New Z velocity magnitude is less than or equal to previous, so it's unlikely to cause a change.
				strategyContext->minZ = strategyContext->minZ + 0.01;
			}
		} else {
			strategyContext->countXStill = 0;
		}

		if ((zVel != 0.0) && (abs(zVel) < strategyContext->minZ)) {
			zVel = zVel >= 0 ? strategyContext->minZ : -strategyContext->minZ;
		}

		static const double maxZVel = 0.5;

		if (abs(zVel) > maxZVel) zVel = zVel >= 0.0 ? maxZVel : -maxZVel;


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

		if ((xVel != 0.0) && (abs(xVel) < strategyContext->minX)) {
			ROS_INFO("override xVel %7.4f with minX: %7.4f", xVel,  strategyContext->minX);
			xVel = xVel >= 0 ? strategyContext->minX : -strategyContext->minX;
		}

		static const double maxXVel = 0.20;

		if (abs(xVel) > maxXVel) xVel = xVel >= 0.0 ? maxXVel : -maxXVel;

		strategyContext->lastX = x;
		strategyContext->lastY = y;

		int xDelta = abs(x - DESIRED_X);
		int yDelta = abs(y - DESIRED_Y);

		if (y > DESIRED_Y) {
			xVel = -0.5;
			ROS_INFO("Gone past Y");
		}
		/*--- End common section */

		cmdVel.linear.x = xVel;
		cmdVel.angular.z = zVel;
		cmdVelPub.publish(cmdVel);

		strategyContext->atHome = (xDelta < DESIRED_X_TOLERANCE) &&
			((yDelta < DESIRED_Y_TOLERANCE) || (y > DESIRED_Y));

		ROS_INFO_STREAM("[GoHome::tick] NearField Need to move towards home, x:"
				<< x
				<< ", y: "
				<< y
				<< ", xVel: "
				<< xVel
				<< ", zVel: "
				<< zVel
				<< ", desired x: "
				<< DESIRED_X
				<< ", xDelta ("
				<< xDelta
				<< ") needs to be under "
				<< DESIRED_X_TOLERANCE
				<< ", desired y: "
				<< DESIRED_Y
				<< ", yDelta ("
				<< yDelta
				<< ") needs to be under "
				<< DESIRED_Y_TOLERANCE
				<< ", minXVel: "
				<< strategyContext->minX
				<< ", minZVel: "
				<< strategyContext->minZ);


		result = RUNNING; // TODO Finish strategy.
	} else if (strategyContext->homeIsVisibleMidField) {
		// Move towards sample using midfield camera.
		publishCurrentStragety(strategyMovingTowardsHomeViaMidfieldCamera);
		strategyContext->movingViaMidfieldCamera = true;

		/*-- Common between GoHome and FetchPrecachedSample --*/
		if (strategyContext->minX < 0.11) {
			// Quick fix.
			strategyContext->minX = 0.11;
		}

		double x = KaimiMidField::Singleton().x();
		double y = KaimiMidField::Singleton().y();
		double zVel = 0.0;
		double xVel = 0.0;
		double desiredX = KaimiMidField::Singleton().cols() / 2.0;

		if (abs(x - desiredX) > DESIRED_X_TOLERANCE) {
			// TODO compute angle rather than pixel offset
			// Need to rotate to center
			zVel = ((desiredX - x) * (0.6 / desiredX));
		}

		xVel = 0.2;
		
		if (abs(strategyContext->lastX - x) < 2) {
			strategyContext->countXStill++;
			// Last Z velocity should have rotated and didn't.
			if (strategyContext->countXStill >= 4) {
				// New Z velocity magnitude is less than or equal to previous, so it's unlikely to cause a change.
				strategyContext->minZ = strategyContext->minZ + 0.01;
			}
		} else {
			strategyContext->countXStill = 0;
		}

		if ((zVel != 0.0) && (abs(zVel) < strategyContext->minZ)) {
			zVel = zVel >= 0 ? strategyContext->minZ : -strategyContext->minZ;
		}

		static const double maxZVel = 0.25;

		if (abs(zVel) > maxZVel) zVel = zVel >= 0.0 ? maxZVel : -maxZVel;

		strategyContext->lastX = x;
		strategyContext->lastY = y;

		int xDelta = abs(x - desiredX);

		strategyContext->cmdVel.linear.x = xVel;
		strategyContext->cmdVel.angular.z = zVel;
		gettimeofday(&strategyContext->periodStart, NULL);


		/*--- End common section */

		ROS_INFO_STREAM("[GoHome::tick] MidField Need to move towards sample, x:"
				<< x
				<< ", y: "
				<< y
				<< ", xVel: "
				<< xVel
				<< ", zVel: "
				<< zVel
				<< ", desired x: "
				<< desiredX
				<< ", xDelta ("
				<< xDelta
				<< ") needs to be under "
				<< DESIRED_X_TOLERANCE);


		result = RUNNING; // TODO Finish strategy.
	} else if (strategyContext->movingViaMidfieldCamera) {
		// Move for 1 second towards home using the midfield camera.
		publishCurrentStragety(strategyMovingTowardsHomeViaMidfieldCamera);
		cmdVelPub.publish(strategyContext->cmdVel);

		double x = KaimiMidField::Singleton().x();
		double y = KaimiMidField::Singleton().y();
		double desiredX = KaimiMidField::Singleton().cols() / 2.0;
		int xDelta = abs(x - desiredX);
		int yDelta = 800 - y;
		bool shouldHaveBeenSeenByNearField = (xDelta < DESIRED_X_TOLERANCE) && (yDelta < DESIRED_Y_TOLERANCE);

		if (shouldHaveBeenSeenByNearField) {
			ROS_WARN("[GoHome::tick] MIDFIELD MOVEMENT SHOULD HAVE CAUSED NEARFIELD TO SEE SAMPLE BY NOW");
			strategyContext->movingViaMidfieldCamera = false;
			result = FAILED;
		} else {
			// Keep this up for one second
			struct timeval now;
			gettimeofday(&now, NULL);
			long seconds = now.tv_sec - strategyContext->periodStart.tv_sec;
			long useconds = now.tv_usec - strategyContext->periodStart.tv_usec;
			double totalElapsedTime = (seconds * 1.0) + (useconds / 1000000.0);
			if (totalElapsedTime > 0.1) {
				strategyContext->movingViaMidfieldCamera = false;
				result = SUCCESS;
			} else {
				// Keep going for one second.
				// ##### TODO Avoid obstacles.
				result = RUNNING;
			}
		}
	} else {
		// Home is not visible.
		publishCurrentStragety(strategyNoHomeSeen);
		// strategyContext->cmdVel.linear.x = 0.2;
		// strategyContext->cmdVel.angular.z = 0.0;
		// cmdVelPub.publish(strategyContext->cmdVel);

		// ##### TODO Move forward blindly.
		// ##### TODO If was previously visible, try to find it again.

		ROS_INFO_STREAM("[GoHome::tick] home is not visible");
		result = FAILED;
	}

	return result;
}

const string GoHome::strategyHasntStarted = "GoHome: Strategy hasn't started";
const string GoHome::strategyMovingTowardsHomeViaMidfieldCamera = "GoHome: Moving towards home via midfield camera";
const string GoHome::strategyMovingTowardsHomeViaNearfieldCamera = "GoHome: Moving towards home via nearfield camera";
const string GoHome::strategyNoHomeSeen = "GoHome:: No target seen";
const string GoHome::strategySuccess = "GoHome: SUCCESS";
const string GoHome::strategyTurning180 = "GoHome: Turnning 180 degrees";

