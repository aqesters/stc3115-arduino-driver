#include <Wire.h>
#include <STC3115.h>

// Inputs and parameters
unsigned long period = 5000;    // time (in ms) between battery measurements 

// instantiate STC3115 object
STC3115 gauge(110, 50, 100);    // capacity, sense resistor, internal resistance
int p = 0;    // index for tracking periods

void setup() {
  // must execute these before constructing STC3115 object
  Serial.begin(9600);     
  Wire.begin();
  delay(50);

  // initialize gauge properly
  gauge.startup();
}

void loop() {
  gauge.check();    // make sure gauge is running
  
  // all measurements needed
  float voltage = gauge.readVoltage();
  float ocv = gauge.readOCV();
  float current = gauge.readCurrent();
  float soc = gauge.readSOC();

  // print measurement results to track
  Serial.print(p);
  Serial.print("\tVoltage: ");
  Serial.print(voltage);
  Serial.print(" V  |  OCV: ");
  Serial.print(ocv);
  Serial.print(" V  |  Current: ");
  Serial.print(current);
  Serial.print(" mA  |  SOC: ");
  Serial.print(soc);
  Serial.println("%");
  Serial.flush();

  delay(period);    // sets measurement frequency
  p++;              // counter
}