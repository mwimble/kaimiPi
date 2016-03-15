#ifndef __DIFF_DRIVE_CONTROLLER
#define __DIFF_DRIVE_CONTROLLER

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/asio.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <wiringPiI2C.h>

using namespace std;
using namespace boost::posix_time;

class DiffDriveController {
private:
	static const int PAUSE_PIN = 12;

	static const int MCP4725_FB_ADDR = 0x62;
	static const int MCP4725_LR_ADDR = 0x63;
	static const int MCP4726_CMD_WRITEDAC = (0x40);  // Writes data to the DAC
	static const int MCP4726_CMD_WRITEDACEEPROM = (0x60);  // Writes data to the DAC and the EEPROM (persisting the assigned value after reset)
	static const unsigned int STOP_VALUE = 2048;

	static float WHEEL_SEPARATION;
    static float WHEEL_RADIUS;
    static float WHEEL_SEPARATION_MULTIPLIER;
    static float WHEEL_RADIUS_MULTIPLIER;
 
	int fbHandle_; // I2C handle for front/back DAC.
	int lrHandle_; // I2C handle for left/right DAC.
	
	string debugStreamName_;

    struct Command {
		double linear;
		double angular;
		ptime timeNsec;

		Command() : linear(0.0), angular(0.0), timeNsec(microsec_clock::local_time()) {}
//		~Command() {}
    };

    boost::lockfree::queue<struct Command, boost::lockfree::capacity<50> > commandQueue_;
    boost::mutex stateLock_;

    ptime timeLastCommandReceived_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_command_;

	void cmdVelCallback(const geometry_msgs::Twist& commandMessage);

	void commandExecutionDoWork();
	boost::thread* commandExecutionThread;

	void commandTimeoutHandler();
	boost::thread* commandTimeoutThread;

	// Set voltage via a DAC.
	// Input:
	//		fd 		Handle to DAC.
	//		voltage Voltage in [0..4095].
	//		persis 	True => write to EEPROM.
	void setVoltage(int fd, int voltage, int persist);

	// Private methods to enforce Singleton pattern.
	DiffDriveController();
	DiffDriveController(DiffDriveController const&) {};
	DiffDriveController& operator=(DiffDriveController const&) {};

	bool commandIsExecuting;

public:
	static DiffDriveController& Singleton();

	// Is the pause switch in effect?
	bool paused();

	// Stop motors.
	void stop();
	
};

#endif