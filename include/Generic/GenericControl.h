/*
 * GenericControl.h
 *
 *  Created on: 5 Mar 2014
 *      Author: Nicoas
 */

#ifndef GENERICCONTROL_H_
#define GENERICCONTROL_H_

#include "Constants.h"

class GenericControl {
public:
	GenericControl(){};
	virtual ~GenericControl(){};

	virtual bool read() = 0;
	virtual bool isCtrlCommandReady() = 0;
	virtual Constants::CtrlCommand::ECtrlCommand getCtrlCommand() = 0;
	virtual void cmdNextStep()=0;
	virtual bool isSimpleKFactorsReady()=0;
	virtual double getSimpleKP()=0;
	virtual double getSimpleKD()=0;

};

#endif /* GENERICCONTROL_H_ */
