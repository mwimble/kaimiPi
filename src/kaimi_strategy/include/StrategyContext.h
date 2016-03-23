#ifndef __STRATEGY_CONTEXT
#define __STRATEGY_CONTEXT

class StrategyContext {
public:
	/*
	* For fetching the precached sample.
	*/
	bool atPrecachedSample;					// True => Arrived at precached sample, ready to retrieve.

	bool precachedSampleFetched;			// True => Have picked up (not necessarily delivered) the precached sample.

	bool precachedSampleIsVeryNear;			// True => While moving towards precached sample, we came very near to it.
};

#endif