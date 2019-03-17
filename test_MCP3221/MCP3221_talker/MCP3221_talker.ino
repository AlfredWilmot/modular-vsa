#include <MCP3221.h>

/* Reference code taken from here: https://github.com/nadavmatalon/MCP3221/blob/master/examples/MCP3221_Basic_Usage/MCP3221_Basic_Usage.ino */

const byte prxml_encdr_addr = 0x4D;

unsigned long timeNow;

MCP3221 proximal_encoder(prxml_encdr_addr);

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  Serial.print(F("\n\nserial is open\n\n"));
  proximal_encoder.setVref(4096);                            // sets voltage reference for the ADC in mV (change as needed)
  proximal_encoder.setVinput(VOLTAGE_INPUT_5V);              // sets voltage input type to be measured (change as needed)
  timeNow = millis();
}

void loop() 
{
  if (millis() - timeNow >= 10) 
  {
    Serial.print(F("reading:\t"));
    Serial.print(proximal_encoder.getData());
    Serial.print(F("\n\n"));  
    timeNow = millis();
  }
}
