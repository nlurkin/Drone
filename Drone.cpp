// Do not remove the include below
#include "Drone.h"
#include <Arduino.h>
#include "MainLoop.h"

MainLoop *mainloop;
//The setup function is called once at startup of the sketch
void setup()
{
	Wire.begin();

	Serial.begin(9600);

	mainloop = new MainLoop();
	// Add your initialization code here
	mainloop->setup();
}

// The loop function is called in an endless loop
void loop()
{
	//Add your repeated code here
	//sensor.exportValueToSerial();
	mainloop->loop();
}
