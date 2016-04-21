#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "/usr/local/include/raspicam/raspicam_cv.h"
#include <camera_info_manager/camera_info_manager.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh("~");
    raspicam::RaspiCam_Cv cap;

    if (!cap.open()) {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    int fps;
    nh.param("fps", fps, 10);
    std::string color_mode = "rgb8";

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 1);
    std::string camera_name = nh.getNamespace();
    camera_info_manager::CameraInfoManager cinfo_(nh, camera_name);

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

// int iLowH = 116;
// int iHighH = 134;

// int iLowS = 96; 
// int iHighS = 255;

// int iLowV = 0;
// int iHighV = 255;

// //Create trackbars in "Control" window
// cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
// cvCreateTrackbar("HighH", "Control", &iHighH, 179);

// cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
// cvCreateTrackbar("HighS", "Control", &iHighS, 255);

// cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
// cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    ros::Rate rate(fps);
    while(ros::ok()) {
        cap.grab();
        Mat imgOriginal;

        //bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        cap.grab();
        bool bSuccess = true;
        cap.retrieve(imgOriginal); // read a new frame from video

        std_msgs::Header header();
        cv_bridge::CvImage imgmsg;
        sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
        imgmsg.header.frame_id = camera_name + "_optical_frame";
        ci.header.frame_id = imgmsg.header.frame_id;
        imgmsg.encoding = sensor_msgs::image_encodings::BGR8;// color_mode;
        imgmsg.image = imgOriginal;
        pub.publish(*imgmsg.toImageMsg());//, ci, ros::Time::now());
        rate.sleep();
        ros::spinOnce();

    // Mat imgHSV;

    // cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    // Mat imgThresholded;

    // inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    // //morphological opening (remove small objects from the foreground)
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    // dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    // //morphological closing (fill small holes in the foreground)
    // dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // imshow("Thresholded Image", imgThresholded); //show the thresholded image
    // imshow("Original", imgOriginal); //show the original image

    // if (waitKey(30) == 27) { //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    //     cout << "esc key is pressed by user" << endl;
    //     break; 
    // }

    }

   return 0;
}