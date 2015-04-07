/*
 * FlightLoop.h
 *
 *  Created on: 7 Apr 2015
 *      Author: Nicoas
 */

#ifndef SRC_FSM_FLIGHTLOOP_H_
#define SRC_FSM_FLIGHTLOOP_H_

#include <Generic/FSMLoop.h>

class FlightLoop: public FSMLoop {
public:
	FlightLoop();
	virtual ~FlightLoop();

	bool processLoop();
	void start();

private:
	enum FSMState {kIDLE};

	FSMState fState;
};

#endif /* SRC_FSM_FLIGHTLOOP_H_ */
