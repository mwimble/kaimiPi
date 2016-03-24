#ifndef __KAIMI_STRATEGY_FN
#define __KAIMI_STRATEGY_FN

#include <string>
#include "StrategyContext.h"

using namespace std;

class KaimiStrategyFn {
private:
	static const char* RESULT_STR[];

public:
	typedef enum {
		FAILED,				// Strategy failed, do not continue.
		FATAL,				// Something is fatally wrong.
		RESTART_LOOP,		// Strategy prempts downstream strategies, go to top of tree.
		RUNNING,			// Strategy is in progress.
		SUCCESS				// Strategy succeeded, continue on.
	} RESULT_T;

	// Name of strategy.
	virtual string name() = 0;

	static string resultToString(RESULT_T result) {
		return string(RESULT_STR[result]);
	}

	// Perform strategy.
	virtual RESULT_T tick(StrategyContext* strategyContext) = 0;
};

#endif
