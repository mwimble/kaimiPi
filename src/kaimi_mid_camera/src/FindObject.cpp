// TODO
// Make singleton.

#include <boost/function.hpp>
#include <boost/bind.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <ctime>
#include <cstdio>

#include "/usr/local/include/raspicam/raspicam_cv.h"
#include <camera_info_manager/camera_info_manager.h>

#include "FindObject.h"

using namespace cv;
using namespace std;

extern FindObject* findObject;

#define TIME_STEPS 0

void FindObject::imageCb(Mat& image) {
	clock_t start;
	double durationCvtColor = 0;
	double durationInRange = 0;
	double durationErode1 = 0;
	double durationDilate1 = 0;
	double durationErode2 = 0;
	double durationDilate2 = 0;
	double durationCopyTo = 0;
	double durationFindLargest = 0;
	double durationShowWIndows = 0;
	double durationFindContours = 0;
	double durationContoursPoly = 0;

    //ROS_INFO("FindObject::imageCb] image.rows: %d, image.cols: %d", image.rows, image.cols);
    if (1 /*image.rows > 60 && image.cols > 60*/) {
    	Mat imgHSV;
    	if (TIME_STEPS) start = clock();
		cvtColor(image, imgHSV, CV_BGR2HSV); // Convert the captured frame from BGR to HSV
		if (TIME_STEPS) durationCvtColor = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;


		Mat imgThresholded;
    	if (TIME_STEPS) start = clock();
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		if (TIME_STEPS) durationInRange = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;




		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		double contourSize;
		Mat tempImage;

    	if (TIME_STEPS) start = clock();
		imgThresholded.copyTo(tempImage);
		durationCopyTo = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;

    	if (TIME_STEPS) start = clock();
		findContours(tempImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		durationFindContours = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;

    	if (TIME_STEPS) start = clock();
		vector<Rect> boundRect( contours.size() );
		vector<vector<Point> > contours_poly( contours.size() );
		durationContoursPoly = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;

		//ROS_INFO("Thresholded iLowH: %d, iHighH: %d, iLowS: %d, iHighS: %d, iLowV: %d, iHighV: %d, countours: %d", iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, contours.size());
		Point2f center;
		ROS_INFO("Found %d contours", contours.size());
		float radius;

    	if (TIME_STEPS) start = clock();
		if (!contours.empty()) {
			// Find largest blob.
			size_t maxBlobIndex = -1;
			int maxBlobSize = 0;
			for (size_t i = 0; i < contours.size(); i++) {
				contourSize = contourArea(contours[i]);
				if ((contourSize > maxBlobSize) && (contourSize >= contourSizeThreshold)) {
					maxBlobIndex = i;
					maxBlobSize = contourSize;
				}
			}

			if (maxBlobIndex != -1) {
				approxPolyDP( Mat(contours[maxBlobIndex]), contours_poly[maxBlobIndex], 3, true );
				boundRect[maxBlobIndex] = boundingRect( Mat(contours_poly[maxBlobIndex]) );
				minEnclosingCircle( (Mat) contours_poly[maxBlobIndex], center, radius );

				double x = center.x;
				double y = center.y;

				int cols = image.cols;
				int rows = image.rows;

				if (showWindows_) {
					Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
					circle(image, center, (int)radius, color, 2, 8, 0 );
				}

				stringstream msg;
				msg << "MidCamera:Found;X:" << x
					<< ";Y:" << y
					<< ";AREA:" << maxBlobSize
					<< ";I:" << maxBlobIndex
					<< ";ROWS:" << image.rows
					<< ";COLS:" << image.cols;
				std_msgs::String message;
				message.data = msg.str();
				midSampleFoundPub_.publish(message);
				ROS_INFO("[FindObject::imageCb] FOUND at x: %7.2f, y: %7.2f, area: %d", x, y, maxBlobSize);
			}
			durationFindLargest = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;
		} else {
			stringstream msg;
			msg << "MidCamera:NotFound;X:0;Y:0;AREA:0;I:0;ROWS:"
				<< image.rows
				<< ";COLS:" << image.cols;
			std_msgs::String message;
			message.data = msg.str();
			midSampleFoundPub_.publish(message);
			ROS_INFO("[FindObject::imageCb] NOT FOUND");
			durationFindLargest = 0;
		}

		if (TIME_STEPS) start = clock();
		if (showWindows_) {
			imshow("[kaimi_mid_camera] Raw Image", image); //show the original image
			imshow("[kaimi_mid_camera] Thresholded Image", imgThresholded); //show the thresholded image
			// ROS_INFO("[FindObject::imageCb] showed images");
			cv::waitKey(25);
		}
		durationShowWIndows = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;
		if (TIME_STEPS) ROS_INFO("durations cvtColor: %7.5f, inRange: %7.5f, erode1: %7.5f, dilate1: %7.5f, dilate2: %7.5f, erode2: %7.5f, findLargest: %7.5f, showWindows: %7.5f, copyTo: %7.5f, findContours: %7.5f, contoursPoly: %7.5f",
			durationCvtColor,
			durationInRange,
			durationErode1,
			durationDilate1,
			durationDilate2,
			durationErode2,
			durationFindLargest,
			durationShowWIndows,
			durationCopyTo,
			durationFindContours,durationContoursPoly);
    }
}


FindObject::FindObject() : it_(nh_),
	iLowH(101),
	iHighH(159),
	iLowS(124),
	iHighS(255),
	iLowV(13),
	iHighV(70),
	contourSizeThreshold(500),
	showWindows_(false) {
//	f = boost::bind(&FindObject::configurationCallback, _1, _2);
//	dynamicConfigurationServer.setCallback(f);

	ros::param::get("~image_topic_name", imageTopicName_);
	ros::param::get("~show_windows", showWindows_);
	ROS_INFO("PARAM image_topic_name: %s", imageTopicName_.c_str());
	ROS_INFO("PARAM show_windows: %d", showWindows_);
	ROS_INFO("[KaimiMidCamera iLowH: %d, iHighH: %d, iLowS: %d, iHighS: %d, iLowV: %d, iHighV: %d", iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);

	//showWindows_ = false;

	midSampleFoundPub_ = nh_.advertise<std_msgs::String>("midSampleFound", 2);
	if (showWindows_) {
		static const char* controlWindowName = "[kaimi_mid_camera] Control";

    	namedWindow("[kaimi_mid_camera] Raw Image", WINDOW_NORMAL);
    	namedWindow("[kaimi_mid_camera] Thresholded Image", WINDOW_NORMAL);

		// Create trackbars in "Control" window
		namedWindow(controlWindowName, CV_WINDOW_AUTOSIZE); //create a window called "Control"
		cvCreateTrackbar("LowH", controlWindowName, &iLowH, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", controlWindowName, &iHighH, 179);

		cvCreateTrackbar("LowS", controlWindowName, &iLowS, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", controlWindowName, &iHighS, 255);

		cvCreateTrackbar("LowV", controlWindowName, &iLowV, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", controlWindowName, &iHighV, 255);

		cvCreateTrackbar("contourSizeThreshold", controlWindowName, &contourSizeThreshold, 2000);
	}

 	std::string camera_name = "midfield_camera";
	camera_info_manager::CameraInfoManager cinfo_(nh_, camera_name);

   	int fps;
	ros::param::get("~fps", fps);
    std::string color_mode = "BGR8";

    raspicam::RaspiCam_Cv cap;
    cap.open();
	ros::Rate rate(fps);
    while(ros::ok()) {
        Mat imgOriginal;

	 	clock_t start;
		double durationGrab;
		double durationRetrieve;
        
        if (TIME_STEPS) start = clock();
        cap.grab();
        durationGrab = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;

        if (TIME_STEPS) start = clock();
        cap.retrieve(imgOriginal); // read a new frame from video
		durationRetrieve = ( std::clock() - start ) / (double)CLOCKS_PER_SEC;

		if (TIME_STEPS) ROS_INFO("Duration grab: %7.5f, retrieve: %7.5f", durationGrab, durationRetrieve);

        // std_msgs::Header header();
        // cv_bridge::CvImage imgmsg;
        // sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
        // imgmsg.header.frame_id = camera_name + "_optical_frame";
        // ci.header.frame_id = imgmsg.header.frame_id;
        // imgmsg.encoding = sensor_msgs::image_encodings::BGR8;// color_mode;
        // imgmsg.image = imgOriginal;
        imageCb(imgOriginal);//, ci, ros::Time::now());
        rate.sleep();
        ros::spinOnce();
	};
}

FindObject& FindObject::Singleton() {
	static FindObject singleton_;
	return singleton_;
}


