#include <SoftwareWire.h>

// SoftwareWire constructor.
// Parameters:
//   (1) pin for the software sda
//   (2) pin for the software scl
//   (3) use internal pullup resistors. Default true. Set to false to disable them.
//   (4) allow the Slave to stretch the clock pulse. Default true. Set to false for faster code.
//
// Using pin 2 (software sda) and 3 (software scl) in this example.
SoftwareWire myWire(2, 11);

/* CODE is a combination of these two:
   AS5600.cpp			(https://github.com/kanestoboi/AS5600)
   SoftI2CMaster.cpp 	(https://github.com/Testato/SoftwareWire)

   These were combined in order to achieve a software-based I2C interface for using two AS5600 on the same MCU.
   The AS5600 slave address cannot be modified, and so using multiple on the same I2C bus requires a multiplexer.
   */

long _msb;
long _lsb;
long _msbMask = 0b00001111;
byte _ANGLEAddressMSB = 0x0E;
byte _ANGLEAddressLSB = 0x0F;
int _AS5600Address = 0x36;

long get_ANGLE(byte registerMSB, byte registerLSB)
{
  _lsb = 0;
  _msb = 0;
  
  myWire.beginTransmission(_AS5600Address);
  myWire.write(registerMSB);
  myWire.endTransmission();
  delay(10);

  myWire.requestFrom(_AS5600Address, 1);

  if(myWire.available() <=1) {
    _msb = myWire.read();
  }

  myWire.requestFrom(_AS5600Address, 1);

  myWire.beginTransmission(_AS5600Address);
  myWire.write(registerLSB);
  myWire.endTransmission();

  if(myWire.available() <=1) {
    _lsb = myWire.read();
  }

  return (_lsb) + (_msb & _msbMask) * 256;
}



void setup()
{
	Serial.begin(9600); // setup serial plotter.
	myWire.begin(); 	// join i2c bus (address optional for master).
}

word val = 0;

void loop()
{
	val = get_ANGLE(_ANGLEAddressMSB, _ANGLEAddressLSB);
	Serial.println(val);
	delay(2);
}
