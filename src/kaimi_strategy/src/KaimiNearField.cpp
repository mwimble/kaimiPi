// TODO
// Make singleton.

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "KaimiNearField.h"

extern KaimiNearField* findObject;

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
	ROS_INFO("KaimiNearField::topicCb, message: %s", msg.data.c_str());
	
	LeftRight leftRight;

	char localStr[strlen(msg.data.c_str()) + 1];
	strcpy(localStr, msg.data.c_str());
	ROS_INFO("KaimiNearField::topicCb localStr: %s", localStr);

	char* keyValPtr = strtok(localStr, ";");
	while (keyValPtr != NULL) {
		// Found key/value pair.
		char keyValue[strlen(keyValPtr) + 1];
		strcpy(keyValue, keyValPtr);
		ROS_INFO("...keyValue: %s", keyValue);

		char* keyPtr = keyValue;
		while ((*keyPtr) && (*keyPtr != ':')) keyPtr++;
		if (*keyPtr) {
			*keyPtr = '\0';
			char key[keyPtr - keyValue];
			keyPtr++;
			strcpy(key, keyValue);
			ROS_INFO("... key: %s", key);
			char* value = keyPtr;
			ROS_INFO("... value: %s", value);
		} else {
			ROS_INFO("!!! missing value");
		}

		keyValPtr = strtok(NULL, ";");
	}
}


KaimiNearField::KaimiNearField() {
	ros::param::param<std::string>("nearfield_topic_name", nearfieldTopicName_, "/nearSampleFound");
	ROS_INFO("PARAM nearfield_topic_name: %s", nearfieldTopicName_.c_str());
	nearfield_sub_ = nh_.subscribe(nearfieldTopicName_.c_str(), 1, &KaimiNearField::topicCb, this);
}
