#include <...\...\Libraries\OurLib.h>

void setup()
{
	Serial.begin(9600);
	qik.init();
}
void loop()
{
	qik.setM0speed(127);
	Serial.print(getRotations());
}
