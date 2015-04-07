#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Communication/SerialOutput.h"
class GenericSensor;
class GenericMotor;
class GenericControl;
class SimpleControl;
class AttitudeLoop;


namespace Constants{
class CtrlCommand{
public:
	enum ECtrlCommand {kNONE,kUSEDEFAULTCALIB, kDOCALIB,kDODEBUG, kGOSTILL};
};
};

extern GenericSensor *sSensor;
extern GenericMotor *sMotor;
extern GenericControl *sControl;
extern SimpleControl *sAltitude;
extern AttitudeLoop *sAttitude;
extern SerialOutput cout;
extern bool bigDebug;

void PRINTOUT(char const* name);
#endif
