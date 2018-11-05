#include <AS5600.h>


AS5600 ams;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	word val = ams.getAngle();
	Serial.println(val);
	delay(20);
}
