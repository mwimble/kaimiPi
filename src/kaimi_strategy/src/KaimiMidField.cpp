// TODO
// Timeout message to indicate not found.

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <pthread.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "KaimiMidField.h"

boost::asio::io_service KaimiMidFieldIoService_;
boost::asio::deadline_timer KaimiMidFieldDeadlineTimer_(KaimiMidFieldIoService_, boost::posix_time::milliseconds(500));

static void * FindObjectTimerRoutine(const boost::system::error_code& /*e*/) {
	ptime now = microsec_clock::local_time();
	time_duration timeSinceLastFound = now - KaimiMidField::Singleton().lastTimeFound();
	long millisecondsSinceLastReport = (long) timeSinceLastFound.total_milliseconds();
	if (millisecondsSinceLastReport > 500) {
		ROS_INFO("[FindObjectTimerRoutine] no object found in last 500 ms");
		KaimiMidField::Singleton().setNotFound();
	}

	KaimiMidFieldDeadlineTimer_.expires_at(KaimiMidFieldDeadlineTimer_.expires_at() + milliseconds(500));
	KaimiMidFieldDeadlineTimer_.async_wait(FindObjectTimerRoutine);
}

void* heartBeatFunction(void* singleton) {
    KaimiMidFieldDeadlineTimer_.async_wait(FindObjectTimerRoutine);
    KaimiMidFieldIoService_.run();
}

// void FindObject::configurationCallback(kaimi_near_camera::kaimi_near_camera_paramsConfig &config, uint32_t level) {
// 	ROS_INFO("Reconfigure Request hue_low: %d, hue_high: %d, saturation_low: %d, saturation high: %d, value_low: %d, value_high: %d, contourSizeThreshold: %d",
// 	         config.hue_low, config.hue_low,
// 	         config.saturation_low, config.saturation_high,
// 	         config.value_low, config.value_high,
// 	         config.contourSizeThreshold);
// 	//setTrackbarPos("LowV", "Control", 13/*config.value_low*/);
// 	if (findObject) {
// 		findObject->iLowH = config.hue_low;
// 		findObject->iHighH = config.hue_high;
// 		findObject->iLowS = config.saturation_low;
// 		findObject->iHighS = config.saturation_high;
// 		findObject->iLowV = config.value_low;
// 		findObject->iHighV = config.value_high;
// 		findObject->contourSizeThreshold = config.contourSizeThreshold;
// 	}
// }

void KaimiMidField::topicCb(const std_msgs::String& msg) {
	// NearCamera:found;R;X:313.49;Y:408.196;AREA:3201;I:0;ROWS:480;COLS:640
	ROS_INFO("[KaimiMidField::topicCb] Message: %s", msg.data.c_str());
	
	char localStr[strlen(msg.data.c_str()) + 1];
	strcpy(localStr, msg.data.c_str());

	char* keyValPtr = strtok(localStr, ";");
	while (keyValPtr != NULL) {
		// Found key/value pair.
		char keyValue[strlen(keyValPtr) + 1];
		strcpy(keyValue, keyValPtr);

		char* keyPtr = keyValue;
		while ((*keyPtr) && (*keyPtr != ':')) keyPtr++;
		if (*keyPtr) {
			*keyPtr = '\0';
			char key[keyPtr - keyValue];
			keyPtr++;
			strcpy(key, keyValue);
			char* value = keyPtr;
			if (strcmp(key, "Found") == 0) {
				if (strcmp(value, "Found")) {
					setFound();
				} else {
					setNotFound();
				}
			} else if (strcmp(key, "NearCamera") == 0) {
				if (strcmp(value, "Found") == 0) {
					found_ = true;
					lastNearFieldReport_ = microsec_clock::local_time();
				} else {
					found_ = false;
				}
			} else if (strcmp(key, "AREA") == 0) {
				area_ = atof(value);
			} else if (strcmp(key, "X") == 0) {
				x_ = atof(value);
			} else if (strcmp(key, "Y") == 0) {
				y_ = atof(value);
			} else if (strcmp(key, "COLS") == 0) {
				cols_ = atol(value);
			} else if (strcmp(key, "ROWS") == 0) {
				rows_ = atol(value);
			}
		} else {
			ROS_INFO("[KaimiMidField::topicCb] !!! missing value");
		}

		keyValPtr = strtok(NULL, ";");
	}

	ROS_INFO("[KaimiMidField::topicCb] area: %f, cols: %d, rows: %d, x: %f, y: %f", area_, cols_, rows_, x_, y_);
}

KaimiMidField::KaimiMidField() {
	ros::param::param<std::string>("nearfield_topic_name", nearfieldTopicName_, "/nearSampleFound");
	ROS_INFO("PARAM nearfield_topic_name: %s", nearfieldTopicName_.c_str());
	nearfield_sub_ = nh_.subscribe(nearfieldTopicName_.c_str(), 1, &KaimiMidField::topicCb, this);

	pthread_t thread;
	int rc = pthread_create(&thread, NULL, heartBeatFunction, this);

	area_ = 0.0;
	cols_ = 0;
	found_ = false;
	rows_ = 0;
	x_ = 0.0;
	y_ = 0.0;
}


KaimiMidField& KaimiMidField::Singleton() {
	static KaimiMidField singleton_;
	return singleton_;
}


