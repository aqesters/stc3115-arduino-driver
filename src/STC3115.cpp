#include "STC3115.h"

// Use EITHER-OR for fixed-width integer types (e.g., uint8_t)
//#include <cstdint>      // from C++ standard library
#include <Arduino.h>    // from Arduino library

// To print debugging messages, set DEBUG to "1" (designed for Arduino IDE)
#define DEBUG 0
#if DEBUG
  #define DEBUG_PRINT(x) do { Serial.println(F(x)); Serial.flush(); delay(100); } while (0)
#else
  #define DEBUG_PRINT(x)  // Does nothing when DEBUG is 0
#endif
// F() macro is used to store strings in flash memory instead of SRAM

// Constructor implementation
STC3115::STC3115(uint16_t capacity = 0, uint8_t rsense = 0, uint16_t resInt = 0) {
  _address = 0b1110000;   // default I2C address
  _capacity = capacity;
  _rsense = rsense;
  _resInt = resInt;         
  //_ram = {0};    // initialize RAM array
}

// Always run on application start-up
void STC3115::startup() 
{
  DEBUG_PRINT("Commencing device startup...");
  
  // Read part ID (1 byte)
  uint8_t partID = read<uint8_t>(REG_ID, 1);

  // Check if part ID matches expected value
  if (partID == 0x14) {

    // Read RAM registers, compare with expected RAM values
    bool needsInit = false;
    for (int i = 0; i < RAM_SIZE; i++) {
      if ( read<uint8_t>(REG_RAM + i, 1) != _ram[i] ) {  
        needsInit = true;
        break;
      }
    }

    if (needsInit) {  // RAM is not in expected state, reset RAM and initialize
      DEBUG_PRINT("Cannot restore from RAM. Resetting RAM...");

      // reset device RAM first
      for (int i = 0; i < RAM_SIZE; i++) {
          write<uint8_t>(0, REG_RAM + i, 1);
      }

      // initialize and run
      initialize();     
      run();
    }

    else {  // RAM is OK, check for loss-of-power events
      DEBUG_PRINT("RAM is OK.");
      uint8_t status = read<uint8_t>(REG_CTRL, 1);
      bool batfail = (status >> 3) & 1;   // 1 if voltage fell below 2.6 V (UVLO)
      bool pordet = (status >> 4) & 1;    // 1 if voltage fell below 2 V (POR threshold)
      
      if (batfail || pordet) {  // battery removal occurred -> initialize
        DEBUG_PRINT("Component lost power in prior operation.");
        write<uint8_t>(0b11, REG_CTRL, 1);  // clear alarm, reset counter
        initialize();   
      }
      else { 
        restore();      // restore to last known state
      }
      run();            // set to run mode
    }
  }
}

// Initialize if recovering last known state is not possible
void STC3115::initialize() 
{
  DEBUG_PRINT("Initializing...");
  
  // read OCV register (2 bytes)
  uint16_t tempOCV = read<uint16_t>(REG_OCV, 2);

  // set parameters for battery model
  setParameters();

  // restore OCV register with saved value
  write<uint16_t>(tempOCV, REG_OCV, 2);
  delay(200);     // wait for SOC register to be ready

  DEBUG_PRINT("Initialization complete.");
}

// Restore to last known battery state using RAM
void STC3115::restore()
{
  DEBUG_PRINT("Restoring to last known battery state...");
  uint16_t tempSOC = read<uint16_t>(RAM_SOC, 2);   // read SOC from RAM
  setParameters();                                 // set battery parameters
  write<uint16_t>(tempSOC, REG_SOC, 2);            // overwrite SOC register with saved value
  DEBUG_PRINT("Restoration complete.");
}

// Change to "Standby" mode, then set parameters for battery model based on datasheet
void STC3115::setParameters() 
{
  DEBUG_PRINT("Setting parameters...");

  // place in standby mode
  uint8_t tempMode = read<uint8_t>(REG_MODE, 1);
  tempMode &= ~(1 << 4);    // set GG_RUN (bit index 4) to 0
  write<uint8_t>(tempMode, REG_MODE, 1);

  // write model parameters to proper registers (or use default settings if "0")
  if (_rsense && _capacity) {   // if both non-zero
    DEBUG_PRINT("Overwriting settings for current sensing and control...");

    // configures coulomb counter ADC (register value)
    uint16_t regCC = _rsense * _capacity / 49.556;            
    write<uint16_t>(regCC, REG_CC_CNF, 2);

    // configures relaxation current threshold
    uint8_t relaxCurrent = _capacity / 10;                                // current threshold in mA
    uint8_t regRelaxCurrent = uint8_t (relaxCurrent * _rsense / 47.04);   // convert current threshold to register value
    write<uint8_t>(regRelaxCurrent, REG_CURRENT_THRES, 1);
  }

  if (_resInt && _capacity) {   // if both non-zero
    DEBUG_PRINT("Overwriting settings for voltage estimation algorithm...");
    uint16_t regVM = _resInt * _capacity / 977.78;            // configures voltage estimation algorithm (register value)
    write<uint16_t>(regVM, REG_VM_CNF, 2);
  }

  // update RAM with new parameters
  updateRAM();
}

// Set component to "Run" mode
void STC3115::run()
{
  DEBUG_PRINT("Switching to \"Run\" mode...");

  // clear alarm bits
  uint8_t tempCtrl = read<uint8_t>(REG_CTRL, 1);
  tempCtrl &= ~(0b11 << 5);     // set bits 5 and 6 to "0"
  write<uint8_t>(tempCtrl, REG_CTRL, 1);

  // set to mixed mode and disable alarm function 
  write<uint8_t>(1 << 4, REG_MODE, 1);     // only bit 4 is "1"    
}

// Switch from "Run" to "Standby" mode
void STC3115::stop() 
{
  // update RAM first
  updateRAM();

  // set GG_RUN bit to 0
  DEBUG_PRINT("Switching to \"Standby\" mode...");
  uint8_t tempMode = read<uint8_t>(REG_MODE, 1);
  tempMode &= ~(1 << 4);
  write<uint8_t>(tempMode, REG_MODE, 1);
}

// Ensure that component is running, or restart if needed -> great for Arduino loop()
void STC3115::check() 
{
  uint8_t tempCtrl = read<uint8_t>(REG_CTRL, 1);
  uint8_t issues = (tempCtrl >> 3) & 0b11;        // examine two bits
  if (issues) {
    DEBUG_PRINT("Power-on issue detected. Restarting...");
    startup();    // run startup routine to take care of issues
  }
  else {
    run();        // make sure we're in run mode
  }
}

// Update device RAM and local variable _ram (Recommended: run periodically for robust device recovery)
void STC3115::updateRAM() 
{ // NOTE: class variable _ram[0:15] corresponds to device registers 32-47
  // We can use the starting RAM register REG_RAM to offset indices

  // test word
  uint8_t testWord = 0b10101010;
  write<uint8_t>(testWord, RAM_TEST, 1);
  _ram[RAM_TEST - REG_RAM] = testWord;

  // SOC
  uint16_t tempSOC = read<uint16_t>(REG_SOC, 2);
  write<uint16_t>(tempSOC, RAM_SOC, 2);
  _ram[RAM_SOC - REG_RAM] = uint8_t (tempSOC & 0xFF);             // first, lower 8 bits
  _ram[RAM_SOC - REG_RAM + 1] = uint8_t ((tempSOC >> 8) & 0xFF);  // then, upper 8 bits

  // OCV
  uint16_t tempOCV = read<uint16_t>(REG_OCV, 2);
  write<uint16_t>(tempOCV, RAM_OCV, 2);
  _ram[RAM_OCV - REG_RAM] = uint8_t (tempOCV & 0xFF);             // first, lower 8 bits
  _ram[RAM_OCV - REG_RAM + 1] = uint8_t ((tempOCV >> 8) & 0xFF);  // then, upper 8 bits

  // CC_CNF
  uint16_t tempCC = read<uint16_t>(REG_CC_CNF, 2);
  write<uint16_t>(tempCC, RAM_CC_CNF, 2);
  _ram[RAM_CC_CNF - REG_RAM] = uint8_t (tempCC & 0xFF);             // first, lower 8 bits
  _ram[RAM_CC_CNF - REG_RAM + 1] = uint8_t ((tempCC >> 8) & 0xFF);  // then, upper 8 bits

  // VM_CNF
  uint16_t tempVM = read<uint16_t>(REG_VM_CNF, 2);
  write<uint16_t>(tempVM, RAM_VM_CNF, 2);
  _ram[RAM_VM_CNF - REG_RAM] = uint8_t (tempVM & 0xFF);             // first, lower 8 bits
  _ram[RAM_VM_CNF - REG_RAM + 1] = uint8_t ((tempVM >> 8) & 0xFF);  // then, upper 8 bits

  // checksum
  uint8_t checksum = 0xFF & (testWord + tempSOC + tempOCV + tempCC + tempVM);
  write<uint8_t>(checksum, RAM_CHECKSUM, 1);
  _ram[RAM_CHECKSUM - REG_RAM] = checksum;
}

// Read battery voltage
float STC3115::readVoltage()
{
  uint16_t rawValue = read<uint16_t>(REG_VOLTAGE, 2);
  float voltage = rawValue * 2.2/1000.0;    // convert to Volts
  return voltage;
}

// Read battery OCV (open-circuit voltage)
float STC3115::readOCV()
{
  uint16_t rawValue = read<uint16_t>(REG_OCV, 2);
  float ocv = rawValue * 0.55/1000.0;       // convert to Volts
  return ocv;
}

// Read battery current
float STC3115::readCurrent() 
{ 
  // NOTE: rawValue is a signed integer because current can be negative
  int16_t rawValue = read<int16_t>(REG_CURRENT, 2);
  float voltage = rawValue * 5.88;       // voltage across sense resistor, in microVolts
  float current = voltage / _rsense;     // current, in mA  [uV / mOhms = mA]
  return current;
}

// Read battery SOC (state of charge)
float STC3115::readSOC()
{
  uint16_t rawValue = read<uint16_t>(REG_SOC, 2);
  float soc = rawValue * 1.0/512.0;     // convert to % charge
  return soc;
}