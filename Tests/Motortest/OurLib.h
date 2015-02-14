#include <PololuQik.h>
#include <SoftwareSerial.h>

PololuQik2s12v10 qik(8, 9, 10); // Object of our Pololu Qik; hooked up to pins 8,9,10
volatile long encM0 = 0;

void initMotors() //call this in your setup function or else no motors for anyone
{
	qik.init();
	attachInterrupt(0, EncoderA, RISING); 
}

int getRotations()
{
	return encA / 4;
}

