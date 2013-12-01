// Do not remove the include below
#include "Drone.h"
#include "AccGyro.h"


AccGyro sensor(0X68);

//The setup function is called once at startup of the sketch
void setup()
{
	// Add your initialization code here
	Wire.begin();

	Serial.begin(9600);
	Serial.println("Drone");
	sensor.setSimulate(true);
}

// The loop function is called in an endless loop
void loop()
{
	//Add your repeated code here
	sensor.exportValueToSerial();
}
