/*
 * StillLoop.h
 *
 *  Created on: 15 Feb 2015
 *      Author: Nicoas
 */

#include "Generic/FSMLoop.h"

#ifndef INCLUDE_STILLLOOP_H_
#define INCLUDE_STILLLOOP_H_

class StillLoop : public FSMLoop{
public:
	StillLoop();
	virtual ~StillLoop();

	bool processLoop();
	void start();

private:
	enum FSMState {kIDLE, kSTABILIZING, kSTABILIZED};

	void stabilize();
	void maintain();

	FSMState fState;
};

#endif /* INCLUDE_STILLLOOP_H_ */
