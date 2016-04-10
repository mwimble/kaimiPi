#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "FetchPrecachedSample.h"
#include "KaimiImu.h"
#include "KaimiMidField.h"
#include "Motion.h"
#include "wiringPi.h"

using namespace std;

FetchPrecachedSample::FetchPrecachedSample() {
	pausedSub = nh.subscribe("basePaused", 1, &FetchPrecachedSample::pausedCallback, this);
	cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	currentStrategyPub = nh.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy = strategyHasntStarted;
	publishCurrentStragety(strategyHasntStarted);

	wiringPiSetupSys();
	// gripperReadHandle = wiringPiI2CSetup(GRIPPER_READ_PIN);
	// gripperWriteHandle = wiringPiI2CSetup(GRIPPER_WRITE_PIN);
	digitalWrite(GRIPPER_WRITE_PIN, 1); // Initial state
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

//	ROS_INFO("[FetchPrecachedSample::pausedCallback] paused: %d", isPaused);
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

KaimiStrategyFn::RESULT_T FetchPrecachedSample::tick() {
	// Desired Y distance from front of plastic plate to back of precached target = 12.75 inches.
	// x: 449, y: 437

	RESULT_T result = FATAL;

	if (!strategyContext.lookingForPrecachedSample) {
		result = SUCCESS;
		return result;
	}

	strategyContext.precachedSampleIsVisibleNearField = KaimiNearField::Singleton().found() && (KaimiNearField::Singleton().y() > 50);
	strategyContext.precachedSampleIsVisibleMidField = /*!strategyContext.movingViaMidfieldCamera &&*/ KaimiMidField::Singleton().found();

	//ROS_INFO("[FetchPrecachedSample::tick] precachedSampleIsVisibleNearField: %d, precachedSampleIsVisibleMidField: %d", strategyContext.precachedSampleIsVisibleNearField, strategyContext.precachedSampleIsVisibleMidField);
	if (strategyContext.needToTurn180) {
		publishCurrentStragety(strategyTurning180);
		ROS_INFO("Turning 180, current yaw: %f, startYaw: %f, abs(diff): %f", KaimiImu::Singleton().yaw(), strategyContext.startYaw, abs(KaimiImu::Singleton().yaw() - strategyContext.startYaw));
		if (abs(KaimiImu::Singleton().yaw() - strategyContext.startYaw) < 3.0) {
			ROS_INFO("[FetchPrecachedSample::tick] Executing 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.5;
			cmdVelPub.publish(cmdVel);
			result = RUNNING;
		} else {
			ROS_INFO("Turning on HOME goal, current yaw: %f, startYaw: %f, abs(diff): %f", KaimiImu::Singleton().yaw(), strategyContext.startYaw, abs(KaimiImu::Singleton().yaw() - strategyContext.startYaw));
			strategyContext.needToTurn180 = false;
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0.0;
			cmdVelPub.publish(cmdVel);
			strategyContext.lookingForHome = true;
			result = SUCCESS;
			if (strategyContext.atPrecachedSample) {
				strategyContext.lookingForPrecachedSample = false;
			}

			ROS_INFO("[FetchPrecachedSample::tick] COMPLETION of 180 turn. yaw: %7.2f", KaimiImu::Singleton().yaw());
		}

		reportedNoFound = false;
		return result;
	}

	if (strategyContext.waitingPauseOff && (digitalRead(GRIPPER_READ_PIN) == 0)) {
		publishCurrentStragety(stragetyWaitingOnArm);
		result = RUNNING;
		reportedNoFound = false;
		return result;
	}

	if (strategyContext.waitingPauseOff) {
		publishCurrentStragety(strategyWaitingForPauseOff);
		if (!isPaused) {
			strategyContext.waitingPauseOff = false;
			strategyContext.needToTurn180 = true;
			digitalWrite(GRIPPER_WRITE_PIN, 1); // Reset
		}

		result = RUNNING;
		reportedNoFound = false;
		return result;
	} else if (strategyContext.waitingPauseOn) {
		publishCurrentStragety(strategyWaitingForPauseOn);
		if (isPaused) {
			strategyContext.waitingPauseOn = false;
			strategyContext.waitingPauseOff = true;
			digitalWrite(GRIPPER_WRITE_PIN, 0); // Start cycle
		}

		result = RUNNING;
		reportedNoFound = false;
		return result;
	} else if (strategyContext.precachedSampleFetched) {
		publishCurrentStragety(strategySuccess);
		ROS_INFO("[FetchPrecachedSample::tick] precachedSampleFetched, SUCCESS");
		result = SUCCESS;
		reportedNoFound = false;
	} else if (strategyContext.atPrecachedSample) {
		// Need to  pick up sample
		ROS_INFO_STREAM("[FetchPrecachedSample::tick] atPrecachedSample");
		publishCurrentStragety(strategyPickingUpSample);
		strategyContext.waitingPauseOn = true;
		//strategyContext.needToTurn180 = true;
		strategyContext.startYaw = KaimiImu::Singleton().yaw();
		result = SUCCESS;
		reportedNoFound = false;
	// } else if (!strategyContext.precachedSampleIsVisibleNearField && strategyContext.precachedSampleFoundNearField) {
	// 	// Sample has been lost, back up a bit.
	// 	cmdVel.linear.x = -0.3;
	// 	cmdVel.angular.z = 0.0;
	// 	cmdVelPub.publish(cmdVel);
	// 	ROS_INFO("[FetchPrecachedSample::tick] Sample lost, backing up");
	} else if (strategyContext.precachedSampleIsVisibleNearField) {
		// Move towards sample using nearfield camera.
		publishCurrentStragety(strategyMovingTowardsSampleViaNearfieldCamera);

		strategyContext.precachedSampleFoundNearField = true;

		float xDelta;
		float yDelta;
		Motion::Singleton().seekViaNearfield(xDelta, yDelta);

		strategyContext.atPrecachedSample = (xDelta < Motion::NEAR_FIELD_DESIRED_X_TOLERANCE) &&
			((yDelta < Motion::NEAR_FIELD_DESIRED_Y_TOLERANCE) || (KaimiNearField::Singleton().y() >= Motion::NEAR_FIELD_DESIRED_Y));

		result = RUNNING; // TODO Finish strategy.
		reportedNoFound = false;
	} else if (strategyContext.precachedSampleIsVisibleMidField) {
		// Move towards sample using midfield camera.
		publishCurrentStragety(strategyMovingTowardsSampleViaMidfieldCamera);
		//strategyContext.movingViaMidfieldCamera = true;

		float xDelta;
		float yDelta;
		Motion::Singleton().seekViaMidfield(xDelta, yDelta);

		result = RUNNING;
		reportedNoFound = false;
	} else {
		// Precached sample is not visible.
		publishCurrentStragety(strategyNoSampleSeen);
		// strategyContext.cmdVel.linear.x = 0.2;
		// strategyContext.cmdVel.angular.z = 0.0;
		// cmdVelPub.publish(strategyContext.cmdVel);

		// ##### TODO Move forward blindly.
		// ##### TODO If was previously visible, try to find it again.

		if (!reportedNoFound) ROS_INFO_STREAM("[FetchPrecachedSample::tick] precached sample is not visible");
		reportedNoFound = true;
		result = FAILED; // TODO Finish strategy.
	}

	return result;
}

const int FetchPrecachedSample::GRIPPER_READ_PIN = 17;
const int FetchPrecachedSample::GRIPPER_WRITE_PIN = 18;

const string FetchPrecachedSample::strategyHasntStarted = "FetchPrecachedSample: Strategy hasn't started";
const string FetchPrecachedSample::strategyMovingTowardsSampleViaMidfieldCamera = "FetchPrecachedSample: Moving towards sample via midfield camera";
const string FetchPrecachedSample::strategyMovingTowardsSampleViaNearfieldCamera = "FetchPrecachedSample: Moving towards sample via nearfield camera";
const string FetchPrecachedSample::strategyNoSampleSeen = "FetchPrecachedSample:: No target seen";
const string FetchPrecachedSample::strategyPickingUpSample = "FetchPrecachedSample: Picking up sample";
const string FetchPrecachedSample::strategySuccess = "FetchPrecachedSample: SUCCESS";
const string FetchPrecachedSample::strategyTurning180 = "FetchPrecachedSample: Turnning 180 degrees";
const string FetchPrecachedSample::strategyWaitingForPauseOff = "FetchPrecachedSample: Waiting for Pause OFF";
const string FetchPrecachedSample::strategyWaitingForPauseOn = "FetchPrecachedSample: Waiting for Pause ON";
const string FetchPrecachedSample::stragetyWaitingOnArm = "FetchPrecachedSample: Waiting on arm cycle";

StrategyContext& FetchPrecachedSample::strategyContext = StrategyContext::Singleton();
