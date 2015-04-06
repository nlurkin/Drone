/*
 * StillLoop.h
 *
 *  Created on: 15 Feb 2015
 *      Author: Nicoas
 */

#ifndef INCLUDE_STILLLOOP_H_
#define INCLUDE_STILLLOOP_H_

class StillLoop {
public:
	enum StillState {kIDLE, kWAITING, kSCANNING, kTAKEOFF, kSTABILIZING, kIDISTURBED, kIMEASUREP, kIMEASUREM,
			kMDISTURBED, kMMEASURES, kMMEASURED, kMBALANCED, kMBALANCES,
			kAPPLY, kLOAD};

	StillLoop();
	virtual ~StillLoop();

	bool processLoop();
	void start();

private:

};

#endif /* INCLUDE_STILLLOOP_H_ */
