#include <boost/bind.hpp>
#include "DiffDriveController.h"
#include <unistd.h>
#include "wiringPi.h"

using namespace boost::posix_time;

DiffDriveController::DiffDriveController() :
	timeLastCommandReceived_(microsec_clock::local_time()) {
	debugStreamName_ = "DiffDriveController";
	
	wiringPiSetupSys();
	pinMode(PAUSE_PIN, INPUT);
	fbHandle_ = wiringPiI2CSetup(MCP4725_FB_ADDR);
	lrHandle_ = wiringPiI2CSetup(MCP4725_LR_ADDR);
	
	sub_command_ = nh_.subscribe("cmd_vel", 1, &DiffDriveController::cmdVelCallback, this);
	commandTimeoutThread = new boost::thread(boost::bind(&DiffDriveController::commandTimeoutHandler, this));
	commandExecutionThread = new boost::thread(boost::bind(&DiffDriveController::commandExecutionDoWork, this));
}

DiffDriveController& DiffDriveController::Singleton() {
	static DiffDriveController singleton_;
	return singleton_;
}

void DiffDriveController::commandTimeoutHandler() {
	while (ros::ok()) {
		ptime currentTime = microsec_clock::local_time();
		time_duration duration = currentTime - timeLastCommandReceived_;
		ROS_INFO_STREAM("commandTimeoutHandler loop"
			<< ", last command @: " << to_simple_string(timeLastCommandReceived_)
			<< ", duration: " << to_simple_string(duration));

		usleep(500000);
	}
}

void DiffDriveController::cmdVelCallback(const geometry_msgs::Twist& commandMessage) {
	if (ros::ok()) {
		if (paused()) {
			ROS_INFO_STREAM("[DiffDriveController::cmdVelCallback] Robot paused, command dropped");
		} else {
			Command command;
			command.angular  = commandMessage.angular.z;
			command.linear = commandMessage.linear.x;
			command.timeNsec = microsec_clock::local_time();
			//ROS_DEBUG_STREAM_NAMED(debugStreamName_,
			ROS_INFO_STREAM("New Command"
				<< ", linear: " << command.linear
				<< ", angular: " << command.angular
				<< ", timeNsec: " << to_simple_string(command.timeNsec));
			if (!commandQueue_.push(command)) {
				ROS_ERROR("UNABLE TO PUSH COMMAND");
			} else {
				timeLastCommandReceived_ = microsec_clock::local_time();
			}
		}
	} else {
		ROS_ERROR_NAMED(debugStreamName_, "Controller is not running. Command ignored");
	}
}

void DiffDriveController::commandExecutionDoWork() {
	while (ros::ok()) {
		if (paused()) {
			ROS_INFO_STREAM("[DiffDriveController::commandExecutionDoWork] Robot paused, no command dequeued");
		} else {
			if (!commandQueue_.empty()) {
				Command command;
				if (commandQueue_.pop(command)) {
					ROS_INFO_STREAM("DiffDriveController::commandExecutionDoWork] deque command. "
						<< "linear: " << command.linear
						<< ", angular: " << command.angular
						<< ", timeNsec: " << to_simple_string(command.timeNsec));
				} else {
					ROS_ERROR("UNEXPECTED POP FAILURE");
				}
			}
		}

		usleep(100000);
	}
}

bool DiffDriveController::paused() {
	return ! digitalRead(PAUSE_PIN);
}

void DiffDriveController::setVoltage(int fd, int voltage, int persist) {
	// 2 byte array to hold 12bit data chunks
	int data[2];

	// limit check voltage
	voltage = (voltage > 4095) ? 4095 : voltage;
	voltage = (voltage < 0) ? 0 : voltage;

	// MCP4725 expects a 12bit data stream in two bytes (2nd & 3rd of transmission)
	data[0] = (voltage >> 8) & 0xFF; // [0 0 0 0 D12 D11 D10 D9 D8] (first bits are modes for our use 0 is fine)
	data[1] = voltage; // [D7 D6 D5 D4 D3 D2 D1 D0]

	// 1st byte is the register
	if (persist) {
		wiringPiI2CWrite(fd, DiffDriveController::MCP4726_CMD_WRITEDACEEPROM);
	} else {
		wiringPiI2CWrite(fd, DiffDriveController::MCP4726_CMD_WRITEDAC);
	}

	// send our data using the register parameter as our first data byte
	// this ensures the data stream is as the MCP4725 expects
	wiringPiI2CWriteReg8(fd, data[0], data[1]);
}

