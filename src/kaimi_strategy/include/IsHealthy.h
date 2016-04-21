#ifndef __IS_HEALTHY
#define __IS_HEALTHY

#include "KaimiStrategyFn.h"

class IsHealthy : public KaimiStrategyFn {
private:
	// Singleton pattern.
	IsHealthy();
	IsHealthy(IsHealthy const&) {};
	IsHealthy& operator=(IsHealthy const&) {}

public:
	RESULT_T tick();

	string name();

	static IsHealthy& Singleton();
};

#endif
