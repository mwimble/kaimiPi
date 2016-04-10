#include <ros/ros.h>
#include <unistd.h>

#include "GoHome.h"
#include "KaimiImu.h"
#include "KaimiMidField.h"
#include "Motion.h"

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

KaimiStrategyFn::RESULT_T GoHome::tick() {
	RESULT_T result = FATAL;

	if (!strategyContext.lookingForHome) {
		result = SUCCESS;
		return result;
		reportedNoFound = false;
	}

	strategyContext.homeIsVisibleNearField = KaimiNearField::Singleton().found();
	strategyContext.homeIsVisibleMidField = /*!strategyContext.movingViaMidfieldCamera &&*/ KaimiMidField::Singleton().found();

	if (strategyContext.needToTurn180) {
		publishCurrentStragety(strategyTurning180);
		if (abs(KaimiImu::Singleton().yaw() - strategyContext.startYaw) < 3.0) {
			ROS_INFO("[GoHome::tick] Executing 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.5;
			cmdVelPub.publish(cmdVel);
			result = RUNNING;
		} else {
			strategyContext.needToTurn180 = false;
			publishCurrentStragety("!!! DONE !!!"); //#####
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.0;
			cmdVelPub.publish(cmdVel);
			strategyContext.lookingForHome = false;
			result = SUCCESS;
			ROS_INFO("[GoHome::tick] COMPLETION of 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
		}

		reportedNoFound = false;
		return result;
	}

	if (strategyContext.atHome) {
		// Need to turn around, for show.
		ROS_INFO_STREAM("[GoHome::tick] atHome");
		strategyContext.needToTurn180 = true;
		strategyContext.startYaw = KaimiImu::Singleton().yaw();
		result = SUCCESS;
		reportedNoFound = false;
	} else if (strategyContext.homeIsVisibleNearField) {
		// Move towards home using nearfield camera.
		publishCurrentStragety(strategyMovingTowardsHomeViaNearfieldCamera);

		float xDelta;
		float yDelta;
		Motion::Singleton().seekViaNearfield(xDelta, yDelta);

		strategyContext.atHome = (xDelta < Motion::NEAR_FIELD_DESIRED_X_TOLERANCE) &&
			((yDelta < Motion::NEAR_FIELD_DESIRED_Y_TOLERANCE) || (KaimiNearField::Singleton().y() >= Motion::NEAR_FIELD_DESIRED_Y));

		result = RUNNING; // TODO Finish strategy.
		reportedNoFound = false;
	} else if (strategyContext.homeIsVisibleMidField) {
		// Move towards sample using midfield camera.
		publishCurrentStragety(strategyMovingTowardsHomeViaMidfieldCamera);
		//strategyContext.movingViaMidfieldCamera = true;

		float xDelta;
		float yDelta;
		Motion::Singleton().seekViaMidfield(xDelta, yDelta);

		result = RUNNING; // TODO Finish strategy.
		reportedNoFound = false;
	} else {
		// Home is not visible.
		publishCurrentStragety(strategyNoHomeSeen);
		// strategyContext.cmdVel.linear.x = 0.2;
		// strategyContext.cmdVel.angular.z = 0.0;
		// cmdVelPub.publish(strategyContext.cmdVel);

		// ##### TODO Move forward blindly.
		// ##### TODO If was previously visible, try to find it again.

		if (!reportedNoFound) ROS_INFO_STREAM("[GoHome::tick] home is not visible");
		reportedNoFound = true;
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
StrategyContext& GoHome::strategyContext = StrategyContext::Singleton();
