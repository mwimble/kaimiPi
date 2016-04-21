
#include <ros/ros.h>
#include <ros/console.h>

#include "opencv2/core/core.hpp"
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <string>

//#include "kaimi_mid_camera/kaimi_mid_camera_paramsConfig.h"

using namespace std;
using namespace cv;

class FindObject {
private:

	ros::NodeHandle nh_;
	string imageTopicName_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;

	FindObject();
	FindObject(FindObject const&) : it_(nh_) {};
	FindObject& operator=(FindObject const&) {};
	static FindObject* singleton;

public:
	static FindObject& Singleton();

};
