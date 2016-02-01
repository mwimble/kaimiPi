#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <time.h>


int main(int argc, char** argv) {
	printf("----START----\n");
	ROS_INFO("Server start\n");
	ros::init(argc, argv, "kaimi_test");
	ros::NodeHandle nh;

	ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	int count = 0;
	ros::Rate r(1);
	while (ros::ok()) {
		ros::spinOnce();
		int eighthSecs = (rand() % 8) + 1;
		ROS_INFO("Loop count: %d, 1/8 secs: %d", count, eighthSecs);
		geometry_msgs::Twist twistMsg;
		twistMsg.linear.x = 1.2;
		twistMsg.linear.y = count++;
		twistMsg.angular.z = eighthSecs;
		twistPublisher.publish(twistMsg);
		//r.sleep();
		timespec t;
		t.tv_sec = eighthSecs / 8;
		t.tv_nsec = 125000000 * (eighthSecs % 8);
		nanosleep(&t, NULL);
	}
}