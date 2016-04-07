#ifndef __STRATEGY_CONTEXT
#define __STRATEGY_CONTEXT

#include <sys/time.h>
#include <geometry_msgs/Twist.h>

class StrategyContext {
public:
	/*
	* For fetching the precached sample.
	*/
	bool atPrecachedSample;						// True => Arrived at precached sample, ready to retrieve.

	bool needToTurn180;							// True => Need to turn 180 degrees.

	bool precachedSampleFetched;				// True => Have picked up (not necessarily delivered) the precached sample.

	bool precachedSampleIsVisibleNearField;		// True => Near field camera sees precached sample.

	bool precachedSampleIsVisibleMidField;		// True => Mid field camera sees precached sample.

	bool movingViaMidfieldCamera;				// True => Moving for a period because of the midfield camera.

	struct timeval periodStart;					// For measuring a period.

	geometry_msgs::Twist cmdVel;				// For use in repeating command.
};

#endif
