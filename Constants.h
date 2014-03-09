#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "SerialOutput.h"
class GenericSensor;
class GenericMotor;
class GenericControl;


namespace Constants{
class CtrlCommand{
public:
	enum ECtrlCommand {kNONE,kDOCALIB,kDODEBUG};
};
};

extern GenericSensor *sSensor;
extern GenericMotor *sMotor;
extern GenericControl *sControl;
extern SerialOutput cout;
extern bool bigDebug;

void PRINTOUT(char const* name);
#endif
