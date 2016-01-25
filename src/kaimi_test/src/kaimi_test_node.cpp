#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv) {
	printf("----START----\n");
	ROS_INFO("Server start\n");
	ros::init(argc, argv, "kaimi_test");
	ros::NodeHandle nh;

	ros::Publisher twistPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	int count = 0;
	ros::Rate r(1);
	while (ros::ok()) {
		ROS_INFO("Loop count: %d", ++count);
		geometry_msgs::Twist twistMsg;
		twistMsg.linear.x = 1.0;
		twistMsg.angular.z = 5.0;
		twistPublisher.publish(twistMsg);
		//ros::spinOnce();
		r.sleep();
	}
}