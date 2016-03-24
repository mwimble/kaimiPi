#ifndef __FETCH_PRECACHED_SAMPLE
#define __FETCH_PRECACHED_SAMPLE

#include "KaimiNearField.h"
#include "KaimiStrategyFn.h"

class FetchPrecachedSample : public KaimiStrategyFn {
private:
	KaimiNearField* kaimiNearField;

	// Singleton pattern.
	FetchPrecachedSample();
	FetchPrecachedSample(FetchPrecachedSample const&) {};
	FetchPrecachedSample& operator=(FetchPrecachedSample const&) {}

public:
	RESULT_T tick(StrategyContext* strategyContext);

	string name();

	static FetchPrecachedSample& Singleton();
};

#endif
