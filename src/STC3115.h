#ifndef STC3115_H
#define STC3115_H

#include <Wire.h>   // for I2C communication

class STC3115 {
  public:
    // Constructor
    STC3115(uint16_t capacity, uint8_t rsense, uint16_t resInt);

    // Battery gauge registers (little-endian format)
    static constexpr uint8_t REG_MODE = 0;              // Mode register
    static constexpr uint8_t REG_CTRL = 1;              // Control and status register
    static constexpr uint8_t REG_SOC = 2;              // Gas gauge relative SOC
    static constexpr uint8_t REG_COUNTER = 4;           // Number of conversions
    static constexpr uint8_t REG_CURRENT = 6;           // Battery current value
    static constexpr uint8_t REG_VOLTAGE = 8;           // Battery voltage value
    static constexpr uint8_t REG_OCV = 13;              // OCV register
    static constexpr uint8_t REG_CC_CNF = 15;           // Coulomb counter gas gauge configuration
    static constexpr uint8_t REG_VM_CNF = 17;           // Voltage gas gauge algorithm parameter
    //static constexpr uint8_t REG_ALARM_SOC = 19;        // SOC alarm level (default = 1%)
    //static constexpr uint8_t REG_ALARM_VOLTAGE = 20;    // Battery low voltage alarm level (default is 3 V)
    static constexpr uint8_t REG_CURRENT_THRES = 21;    // Current threshold for the relaxation counter
    static constexpr uint8_t REG_ID = 24;               // Part type ID = 14h
    static constexpr uint8_t REG_RAM = 32;             // starting register for 16 RAM bytes

    // Mapping for RAM storage (device registers 32-47)
    static constexpr uint8_t RAM_TEST = 32;
    static constexpr uint8_t RAM_SOC = 33;
    static constexpr uint8_t RAM_OCV = 35;
    static constexpr uint8_t RAM_CC_CNF = 37;
    static constexpr uint8_t RAM_VM_CNF = 39;
    static constexpr uint8_t RAM_CHECKSUM = 41;
    static constexpr uint8_t RAM_SIZE = RAM_CHECKSUM - REG_RAM;

    // Regular functions
    void startup();
    void initialize();
    void restore();
    void setParameters();
    void updateRAM();
    void run();
    void stop();
    void check();
    float readVoltage();
    float readOCV();
    float readCurrent();
    float readSOC();

    // Template functions for I2C read/write operations
    // use T to specify how to interpret I/O (e.g., unsigned vs signed int)
    template <typename T>
    T read(uint8_t startRegister, uint8_t numBytes);

    template <typename T>
    void write(T command, uint8_t startRegister, uint8_t numBytes);

  private:    // Internal members to store state, if necessary
    uint8_t _address;                    // I2C address 
    uint16_t _capacity;                  // battery capacity in mAh
    uint8_t _rsense;                     // current sense resistor in mOhms     
    uint16_t _resInt;                    // average internal battery resistance in mOhms
    uint8_t _ram[RAM_SIZE] = {0};        // known RAM values
    bool _debug;                         // debug flag
};

// I2C read operation (max read: 2 bytes)
template <typename T>
T STC3115::read(uint8_t startRegister, uint8_t numBytes) 
{ 
  uint16_t rawValue = 0;   // max: 2 bytes worth  

  // start I2C comms with device
  Wire.beginTransmission(_address);  

  // set target register for reading
  Wire.write(startRegister);
  Wire.endTransmission();

  // read contents from consecutive registers (little-endian format)
  Wire.requestFrom(_address, numBytes); 
  for (int i = 0; i < numBytes; i++) {    
    uint8_t nextByte = Wire.read();
    rawValue |= uint16_t (nextByte << (8*i));    // shift new byte into proper position
    // device auto-increments register after each read
  }

  // returned in specified format
  return static_cast<T>(rawValue);
};

// I2C write operation
template <typename T>
void STC3115::write(T command, uint8_t startRegister, uint8_t numBytes) 
{ 
  // start I2C comms with device
  Wire.beginTransmission(_address);

  // write one byte to each consecutive register
  Wire.write(startRegister);      
  for (int i = 0; i < numBytes; i++) { 
    Wire.write(command & 0xFF);   // send lower 8 bits
    command = command >> 8;       // shift to prepare next byte
    // device auto-increments register after each write
  }
  Wire.endTransmission();
};

#endif  // STC3115_H