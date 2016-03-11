
#include <ros/ros.h>
#include <ros/console.h>
 #include "std_msgs/String.h"

#include <dynamic_reconfigure/server.h>
#include <string>

//#include "kaimi_near_camera/kaimi_near_camera_paramsConfig.h"

using namespace std;

class KaimiNearField {
public:
	enum LeftRight {
		FAR_LEFT,
		LEFT,
		CENTER,
		RIGHT,
		FAR_RIGHT,
	};

	static KaimiNearField* Singleton();

private:

	ros::NodeHandle nh_;
	string nearfieldTopicName_;
	ros::Subscriber nearfield_sub_;

	void topicCb(const std_msgs::String& msg);

	// dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig> dynamicConfigurationServer;
	// dynamic_reconfigure::Server<kaimi_near_camera::kaimi_near_camera_paramsConfig>::CallbackType f;

	// static void configurationCallback(kaimi_near_camera::kaimi_near_camera_paramsConfig &config, uint32_t level);

	KaimiNearField() {};
	KaimiNearField(KaimiNearField const&) {};
	KaimiNearField& operator=(KaimiNearField const&) {};
	static KaimiNearField* singleton;

	static LeftRight leftRight_;

};