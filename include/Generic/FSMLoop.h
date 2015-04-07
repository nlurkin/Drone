/*
 * FSMLoop.h
 *
 *  Created on: 7 Apr 2015
 *      Author: Nicoas
 */

#ifndef SRC_FSM_FSMLOOP_H_
#define SRC_FSM_FSMLOOP_H_

class FSMLoop {
public:
	FSMLoop() {};
	virtual ~FSMLoop() {};

	virtual bool processLoop() = 0;
	virtual void start() = 0;
};

#endif /* SRC_FSM_FSMLOOP_H_ */
