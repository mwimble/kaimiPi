#include <ros/ros.h>
#include <ros/console.h>

#include "FetchPrecachedSample.h"
#include "GoHome.h"
#include "IsHealthy.h"
#include "KaimiImu.h"
#include "KaimiMidField.h"
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
	ros::init(argc, argv, "kaimi_strategy_node");
	KaimiNearField& kaimiNearField = KaimiNearField::Singleton();
	KaimiMidField& kaimiMidField = KaimiMidField::Singleton();
	KaimiImu& kaimiImu = KaimiImu::Singleton();

	ros::Rate rate(10); // Loop rate

	StrategyContext* strategyContext = new StrategyContext();
	strategyContext->lookingForPrecachedSample = true;
	behaviors.push_back(&IsHealthy::Singleton());
	behaviors.push_back(&FetchPrecachedSample::Singleton());
	behaviors.push_back(&GoHome::Singleton());

	while (ros::ok()) {
		try { // Emplement Sequence behavior
			rate.sleep();
			ros::spinOnce();

			//ROS_INFO_STREAM("--- ---- ---- ---- Begin of strategy loop ---- ---- ---- ----");
			for(vector<KaimiStrategyFn*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
				KaimiStrategyFn::RESULT_T result = ((*it)->tick)(strategyContext);
				if (result == KaimiStrategyFn::RESTART_LOOP) {
					//ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", RESTART_LOOP result, restarting");
					throw new StrategyException("RESTART_LOOP");
				}

				if (result == KaimiStrategyFn::FATAL) {
					//ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", FATAL result, exiting");
					return -1;
				}

				if (result == KaimiStrategyFn::RUNNING) {
					//ROS_INFO_STREAM("[kaimi_strategy_node] function " << ((*it)->name()) << ", RUNNING, restarting");
					throw new StrategyException("RESTART_LOOP");
				}

				if (result == KaimiStrategyFn::SUCCESS) {
					//ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", SUCCESS result, continuing");
					continue;
				}

				if (result == KaimiStrategyFn::FAILED) {
					//ROS_INFO_STREAM("[kaimi_strategy_node] function: " << ((*it)->name()) << ", FAILED result, aborting");
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

