#ifndef __KAIMI_IMU
#define __KAIMI_IMU

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "std_msgs/String.h"

#include <dynamic_reconfigure/server.h>
#include <string>

//#include "kaimi_near_camera/kaimi_near_camera_paramsConfig.h"

using namespace std;
using namespace boost::posix_time;

class KaimiImu {
public:
	static KaimiImu& Singleton();

	double yaw() { return yaw_; }

private:

	ros::NodeHandle nh_;
	string topicName_;
	ros::Subscriber imu_sub_;

	void topicCb(const sensor_msgs::Imu& msg);

	// dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig> dynamicConfigurationServer;
	// dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig>::CallbackType f;

	// static void configurationCallback(kaimi_near_camera::kaimi_near_camera_paramsConfig &config, uint32_t level);

	KaimiImu();
	KaimiImu(KaimiImu const&) {};
	KaimiImu& operator=(KaimiImu const&) {};

	double yaw_;
};

#endif
