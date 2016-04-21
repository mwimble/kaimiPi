// TODO
// Timeout message to indicate not found.

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tf/transform_datatypes.h>

#include "KaimiImu.h"

void KaimiImu::topicCb(const sensor_msgs::Imu& msg) {
	tf::Quaternion qq;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(msg.orientation, qq);
	tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
	//ROS_INFO("[KaimiImu::topicCb] roll: %7.2f, pitch: %7.2f, yaw: %7.2f", roll, pitch, yaw);
	yaw_ = yaw;
}

KaimiImu::KaimiImu() {
	ros::param::param<std::string>("imu_topic_name", topicName_, "imu");
	ROS_INFO("PARAM imu_topic_name: %s", topicName_.c_str());
	imu_sub_ = nh_.subscribe(topicName_.c_str(), 1, &KaimiImu::topicCb, this);

	yaw_ = 0.0;
}


KaimiImu& KaimiImu::Singleton() {
	static KaimiImu singleton_;
	return singleton_;
}


