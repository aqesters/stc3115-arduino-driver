# STC3115 Battery Gauge Library

Arduino library to interface with the STC3115 battery gauge IC over I2C.

## Features

- Read battery voltage, current, open circuit voltage (OCV), and state of charge (SOC).
- Simple class-based device driver compatible with Arduino IDE and Arduino CLI.
- Example sketch included.

## Installation

1. Download or clone this repository.
2. Copy the `STC3115` folder into your Arduino `libraries` directory. The typical directory is one of the following.
   - On Windows: `Documents\Arduino\libraries`
   - On macOS/Linux: `~/Documents/Arduino/libraries`
3. Restart the Arduino IDE.
4. Include the library in your sketches with `#include <STC3115.h>`.

## Usage

The code below creates an `STC3115` object and uses it to report the "% charge" of the battery (0-100%) every 5 seconds. 
Refer to the datasheet in the `documents` folder to learn how to properly design the schematic and PCB layout for the STC3115 device.

```cpp
#include <Wire.h>
#include <STC3115.h>

// Arguments: battery capacity (mAh), sense resistor (mOhm), battery internal resistance (mOhm)
STC3115 gauge(110, 50, 100); 

void setup() {
    Serial.begin(9600);
    Wire.begin();
    gauge.startup();  // initializes device
}

// Once set up, devie can be used to read battery levels via state of charge (SOC)
void loop() {
    gauge.check();    // ensures device is running
    float soc = gauge.readSOC();
    Serial.print("SOC: ");
    Serial.print(soc);
    Serial.println("%");
    delay(5000);  // 5 seconds between measurements
}
```

