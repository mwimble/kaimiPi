#include <ros/ros.h>
#include <ros/console.h>

#include "FetchPrecachedSample.h"
#include "IsHealthy.h"
#include "KaimiNearField.h"
#include "KaimiStrategyFn.h"
#include "StrategyContext.h"
#include "StrategyException.h"

int debug_last_message = 0; // So that we only emit messages when things change.
		
void logIfChanged(int id, const char* message) {
	if (id != debug_last_message) {
		ROS_INFO_STREAM(message);
		debug_last_message = id;
	}	
}

vector<KaimiStrategyFn*> behaviors;

int main(int argc, char** argv) {
	static const double VEL_FAR_LEFT = 0.4;
	static const double VEL_LEFT = 0.3;
	static const double VEL_FAR_RIGHT = -0.4;
	static const double VEL_RIGHT = -0.3;

	ros::init(argc, argv, "kaimi_strategy_node");
	ros::NodeHandle nh;
	ros::Publisher cmdVelPub;
	geometry_msgs::Twist cmdVel;
	KaimiNearField& kaimiNearField = KaimiNearField::Singleton();

	ros::Rate rate(20); // Loop rate

	// If near field not found, did we previously see it as we were
	// advancing very near to it?
	// bool wasAdvancingOnVeryNear = false;

	// bool printedNoSampleFound = false;

	cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	StrategyContext* strategyContext = new StrategyContext();
	behaviors.push_back(&IsHealthy::Singleton());
	behaviors.push_back(&FetchPrecachedSample::Singleton());

	while (ros::ok()) {
		try { // Emplement Sequence behavior
			rate.sleep();
			ros::spinOnce();

			for(vector<KaimiStrategyFn*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
				KaimiStrategyFn::RESULT_T result = ((*it)->tick)(strategyContext);
				if (result == KaimiStrategyFn::RESTART_LOOP) {
					ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", RESTART_LOOP result, restarting");
					throw new StrategyException("RESTART_LOOP");
				}

				if (result == KaimiStrategyFn::FATAL) {
					ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", FATAL result, exiting");
					return -1;
				}

				if (result == KaimiStrategyFn::RUNNING) {
					ROS_INFO_STREAM("[kaimi_strategy_node] function " << ((*it)->name()) << ", RUNNING, restarting");
					throw new StrategyException("RESTART_LOOP");
				}

				if (result == KaimiStrategyFn::SUCCESS) {
					ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", SUCCESS result, continuing");
					continue;
				}

				if (result == KaimiStrategyFn::FAILED) {
					ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", FAILED result, aborting");
					break;
				}
			}
		} catch(StrategyException* e) {
			//ROS_INFO_STREAM("[kaimi_strategy_node] StrategyException: " << e->what());
			// Do nothing.
		}
	}

	return 0;
}

