#ifndef __KAIMI_NEAR_FIELD
#define __KAIMI_NEAR_FIELD

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "std_msgs/String.h"

#include <dynamic_reconfigure/server.h>
#include <string>

//#include "kaimi_near_camera/kaimi_near_camera_paramsConfig.h"

using namespace std;
using namespace boost::posix_time;

class KaimiNearField {
public:
	static KaimiNearField& Singleton();

	double area() { return area_; }

	int cols() { return cols_; }

	bool found() { return found_; }

	ptime lastTimeFound() { return lastFieldReport_; }

	int rows() { return rows_; }

	void setFound() {
		found_ = true;
	}
	void setNotFound() {
		found_ = false;
	}

	double x() { return x_; }

	double y() { return y_; }

private:

	ros::NodeHandle nh_;
	string nearfieldTopicName_;
	ros::Subscriber nearfield_sub_;

	void topicCb(const std_msgs::String& msg);

	// dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig> dynamicConfigurationServer;
	// dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig>::CallbackType f;

	// static void configurationCallback(kaimi_near_camera::kaimi_near_camera_paramsConfig &config, uint32_t level);

	KaimiNearField();
	KaimiNearField(KaimiNearField const&) {};
	KaimiNearField& operator=(KaimiNearField const&) {};

	double area_;
	int cols_;
	bool found_;
	ptime lastFieldReport_;
	int rows_;
	double x_;
	double y_;
};

#endif
