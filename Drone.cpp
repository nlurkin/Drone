// Do not remove the include below
#include "Drone.h"
#include "AccGyro.h"

AccGyro sensor(0x68);

//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
	Wire.begin();
	Serial.begin(9600);

	Serial.println("Wire initialized.");

	sensor.init();
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
	sensor.exportTeaPot();
	//sensor.exportValueToSerial();
}
