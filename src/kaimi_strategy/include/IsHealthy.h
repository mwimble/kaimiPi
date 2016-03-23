#include "KaimiStrategyFn.h"

class IsHealthy : public KaimiStrategyFn {
	RESULT_T tick(StrategyContext* strategyContext);

	string name();
};