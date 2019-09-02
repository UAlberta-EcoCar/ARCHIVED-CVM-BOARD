

#include <Wire.h> //Include Wire Code Library 

void setup() {
  Serial.begin (115200); // Start Serial Communication at baud rate 115200
  while (!Serial)  
    {
    }

  Serial.println ();
  Serial.println ("I2C Address Scanner. Scanning ...");
  byte count = 0;


    Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found i2c Device Address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}  // end of setup

void loop() {}
