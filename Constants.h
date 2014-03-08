#ifndef CONSTANTS_H
#define CONSTANTS_H

class GenericSensor;
class GenericMotor;
class GenericControl;


namespace Constants{
class CtrlCommand{
public:
	enum ECtrlCommand {kNONE,kDOCALIB};
};
};

static GenericSensor *sSensor;
static GenericMotor *sMotor;
static GenericControl *sControl;
#endif
