// TODO
// Make singleton.

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "/usr/local/include/raspicam/raspicam_cv.h"
#include <camera_info_manager/camera_info_manager.h>

#include "FindObject.h"

using namespace cv;
using namespace std;

extern FindObject* findObject;

FindObject::FindObject() : it_(nh_)	{
//	dynamicConfigurationServer.setCallback(f);

	ros::param::get("~image_topic_name", imageTopicName_);
	ROS_INFO("PARAM image_topic_name: %s", imageTopicName_.c_str());

	image_pub_ = it_.advertise(imageTopicName_, 4);

	std::string camera_name = "midfield_camera";
	camera_info_manager::CameraInfoManager cinfo_(nh_, camera_name);

    int fps;
	ros::param::get("~fps", fps);
    //nh_.param("fps", fps, 10);
    std::string color_mode = "BGR8";

    //image_transport::ImageTransport it(nh);
    // std::string camera_name = nh.getNamespace();
    // camera_info_manager::CameraInfoManager cinfo_(nh, camera_name);

    raspicam::RaspiCam_Cv cap;
    cap.open();
	ros::Rate rate(fps);
    while(ros::ok()) {
        Mat imgOriginal;

        cap.grab();
        cap.retrieve(imgOriginal); // read a new frame from video
	    try {
	        std_msgs::Header header();
	        cv_bridge::CvImage imgmsg;
	        sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
	        imgmsg.header.frame_id = camera_name + "_optical_frame";
	        ci.header.frame_id = imgmsg.header.frame_id;
	        imgmsg.encoding = sensor_msgs::image_encodings::BGR8;// color_mode;
	        imgmsg.image = imgOriginal;
	        image_pub_.publish(*imgmsg.toImageMsg());;
	    } catch (cv_bridge::Exception& e) {
	      ROS_ERROR("[FindObject::imageCb] exception: %s", e.what());
	      return;
	    }

        rate.sleep();
        ros::spinOnce();
	};
}

FindObject& FindObject::Singleton() {
	static FindObject singleton_;
	return singleton_;
}


