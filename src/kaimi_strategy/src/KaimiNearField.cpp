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

#include "KaimiNearField.h"

boost::asio::io_service kaimiNearFieldIoService_;
boost::asio::deadline_timer kaimiNearFieldDeadlineTimer_(kaimiNearFieldIoService_, boost::posix_time::milliseconds(500));

static void * FindObjectTimerRoutine(const boost::system::error_code& /*e*/) {
	ptime now = microsec_clock::local_time();
	time_duration timeSinceLastFound = now - KaimiNearField::Singleton().lastTimeFound();
	long millisecondsSinceLastReport = (long) timeSinceLastFound.total_milliseconds();
	if (millisecondsSinceLastReport > 500) {
		ROS_INFO("[FindObjectTimerRoutine] no object found in last 500 ms");
		KaimiNearField::Singleton().setNotFound();
	}

	kaimiNearFieldDeadlineTimer_.expires_at(kaimiNearFieldDeadlineTimer_.expires_at() + milliseconds(500));
	kaimiNearFieldDeadlineTimer_.async_wait(FindObjectTimerRoutine);
}

void* heartBeatFunction(void* singleton) {
    kaimiNearFieldDeadlineTimer_.async_wait(FindObjectTimerRoutine);
    kaimiNearFieldIoService_.run();
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

void KaimiNearField::topicCb(const std_msgs::String& msg) {
	// NearCamera:Found;LEFT-RIGHT:LR OK;FRONT-BACK:VERY NEAR;X:313.49;Y:408.196;AREA:3201;I:0;ROWS:480;COLS:640
	ROS_INFO("[KaimiNearField::topicCb] Message: %s", msg.data.c_str());
	
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
			} else if (strcmp(key, "LEFT-RIGHT") == 0) {
				if (strcmp(value, "FAR LEFT") == 0) {
					leftRight_ = FAR_LEFT;
				} else if (strcmp(value, "LEFT") == 0) {
					leftRight_ = LEFT;
				} else if (strcmp(value, "LR OK") == 0) {
					leftRight_ = CENTER;
				} else if (strcmp(value, "RIGHT") == 0) {
					leftRight_ = RIGHT;
				} else if (strcmp(value, "FAR RIGHT") == 0) {
					leftRight_ = FAR_RIGHT;
				} else {
					leftRight_ = CENTER;
				}
			} else if (strcmp(key, "FRONT-BACK") == 0) {
				if (strcmp(value, "VERY FAR AWAY") == 0) {
					farNear_ = VERY_FAR_AWAY;
				} else if (strcmp(value, "FAR AWAY") == 0) {
					farNear_ = FAR_AWAY;
				} else if (strcmp(value, "NEAR") == 0) {
					farNear_ = NEAR;
				} else if (strcmp(value, "VERY NEAR") == 0) {
					farNear_ = VERY_NEAR;
				} else {
					farNear_ = VERY_NEAR;
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
			ROS_INFO("[KaimiNearField::topicCb] !!! missing value");
		}

		keyValPtr = strtok(NULL, ";");
	}

	ROS_INFO("[KaimiNearField::topicCb] area: %f, cols: %d, farNear: %s, leftRight: %s, rows: %d, x: %f, y: %f", area_, cols_, FAR_NEAR_TO_STRING[farNear_], LEFT_RIGHT_TO_STRING[leftRight_], rows_, x_, y_);
}

KaimiNearField::KaimiNearField() {
	ros::param::param<std::string>("nearfield_topic_name", nearfieldTopicName_, "/nearSampleFound");
	ROS_INFO("PARAM nearfield_topic_name: %s", nearfieldTopicName_.c_str());
	nearfield_sub_ = nh_.subscribe(nearfieldTopicName_.c_str(), 1, &KaimiNearField::topicCb, this);

	pthread_t thread;
	int rc = pthread_create(&thread, NULL, heartBeatFunction, this);

	area_ = 0.0;
	cols_ = 0;
	found_ = false;
	farNear_ = VERY_NEAR;
	leftRight_ = CENTER;
	rows_ = 0;
	x_ = 0.0;
	y_ = 0.0;
}


KaimiNearField& KaimiNearField::Singleton() {
	static KaimiNearField singleton_;
	return singleton_;
}


const char* KaimiNearField::LEFT_RIGHT_TO_STRING[5] = {
	"FAR_LEFT",
	"LEFT",
	"CENTER",
	"RIGHT",
	"FAR_RIGHT"
};

const char* KaimiNearField::FAR_NEAR_TO_STRING[4] = {
	"VERY_FAR_AWAY",
	"FAR_AWAY",
	"NEAR",
	"VERY_NEAR"
};
