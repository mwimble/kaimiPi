#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "test_goals");

	ros::NodeHandle *rosNode = new ros::NodeHandle(); //### namespace
    ros::Publisher visPub = rosNode->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";

    static const float pi = 3.14159;
    float orientation = 0.0;

    for (int i = 0; i < 4; i++) {
        goal.target_pose.header.stamp = ros::Time::now();
        switch (i) {
        case 0:
            goal.target_pose.pose.position.x = 0.5;
            goal.target_pose.pose.position.y = 0.0;
            orientation = pi / 2;
            break;

        case 1:
            goal.target_pose.pose.position.x = 0.5;
            goal.target_pose.pose.position.y = 0.5;
            orientation = pi;
            break;

        case 2:
            goal.target_pose.pose.position.x = 0.0;
            goal.target_pose.pose.position.y = 0.5;
            orientation = (3 * pi) / 2;
            break;

        case 3:
            goal.target_pose.pose.position.x = 0.0;
            goal.target_pose.pose.position.y = 0.0;
            orientation = 0.0;
            break;

        }

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(orientation );

        goal.target_pose.pose.orientation.x = odom_quat.x;
        goal.target_pose.pose.orientation.y = odom_quat.y;
        goal.target_pose.pose.orientation.z = odom_quat.z;
        goal.target_pose.pose.orientation.w = odom_quat.w;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "testWaypoints";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goal.target_pose.pose.position.x;
        marker.pose.position.y = goal.target_pose.pose.position.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = odom_quat.x;
        marker.pose.orientation.y = odom_quat.y;
        marker.pose.orientation.z = odom_quat.z;
        marker.pose.orientation.w = odom_quat.w;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.27;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        visPub.publish( marker );
        
        ROS_INFO("Sending goal [%d] orientation: %f", i, orientation);
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, the base moved 1 meter forward");
        } else {
            ROS_INFO("The base failed to move forward 1 meter for some reason");
            exit(0);
        }
    }


    return 0;
}