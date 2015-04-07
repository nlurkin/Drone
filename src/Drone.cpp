// Do not remove the include below
#include "Drone.h"
#include <arduino/Arduino.h>
#include "FSM/MainLoop.h"

#include "../TestKalman.h"

MainLoop *mainloop;
//TestKalman *test;
//The setup function is called once at startup of the sketch
void setup()
{
	Wire.begin();

	Serial.begin(9600);

	mainloop = new MainLoop();
	// Add your initialization code here
	mainloop->start();
	//test = new TestKalman();
	//test->test1();
}

// The loop function is called in an endless loop
void loop()
{
	//Add your repeated code here
	//sensor.exportValueToSerial();
	mainloop->processLoop();

}
