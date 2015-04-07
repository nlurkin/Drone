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
	enum StillState {kIDLE, kSTABILIZING, kSTABILIZED};

	StillLoop();
	virtual ~StillLoop();

	bool processLoop();
	void start();

private:
	void stabilize();
	void maintain();

	StillState fState;
};

#endif /* INCLUDE_STILLLOOP_H_ */
