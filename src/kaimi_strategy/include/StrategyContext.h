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

	bool homeIsVisibleNearField;				// True => Near field camera sees home.

	bool homeIsVisibleMidField;					// True => Mid field camera sees home.

	double lastX;								// Used to adjust minZ.

	double lastXVel;							// Last posted xVel.

	double lastY;								// Used to adjust minX.

	double lastZVel;							// Last posted zVel.

	bool lookingForHome;						// True => Trying to return home.

	bool lookingForPrecachedSample;				// True => Trying to find the precached sample;

	double minX;								// Used to dynamically adjust min orientation cmd_vel.

	double minZ;								// Used to dynamically adjust min angular cmd.vel.

	bool movingViaMidfieldCamera;				// True => Moving for a period because of the midfield camera.

	bool needToTurn180;							// True => Need to turn 180 degrees.

	struct timeval periodStart;					// For measuring a period.

	bool precachedSampleFetched;				// True => Have picked up (not necessarily delivered) the precached sample.

	bool precachedSampleIsVisibleNearField;		// True => Near field camera sees precached sample.

	bool precachedSampleIsVisibleMidField;		// True => Mid field camera sees precached sample.

	geometry_msgs::Twist cmdVel;				// For use in repeating command.
};

#endif
