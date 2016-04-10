#ifndef __KAIMI_STRATEGY_FN
#define __KAIMI_STRATEGY_FN

#include <ros/ros.h>
#include <string>
#include "StrategyContext.h"

using namespace std;

class KaimiStrategyFn {
private:
	static const char* RESULT_STR[];

public:
	typedef enum {
		UNUSED_START = 0,		// Do not use, must be first element.
		FAILED,				// Strategy failed, do not continue.
		FATAL,				// Something is fatally wrong.
		RESTART_LOOP,		// Strategy prempts downstream strategies, go to top of tree.
		RUNNING,			// Strategy is in progress.
		SUCCESS,			// Strategy succeeded, continue on.
		UNUSED_END			// Do not use, must be last element.
	} RESULT_T;

	// Name of strategy.
	virtual string name() = 0;

	static string resultToString(RESULT_T result) {
		if ((result <= UNUSED_START) || (result >= UNUSED_END)) {
			return "BAD result code";
		} else {
			return string(RESULT_STR[result]);
		}
	}

	// Perform strategy.
	virtual RESULT_T tick() = 0;
};

#endif
