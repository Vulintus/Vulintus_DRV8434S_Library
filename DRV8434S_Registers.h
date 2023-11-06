/*  DRV8434S_Registers.h - copyright Vulintus, Inc., 2023

    Register addresses and settings for the Texas Instruments DRV8434S stepper
    driver.

    UPDATE LOG:
      2023-11-04 - Drew Sloan - Library first created.

*/

#ifndef DRV8434S_REGISTERS_H
#define DRV8434S_REGISTERS_H


// REGISTER ADDRESSES ************************************************************************************************// 
enum DRV8434S_REG_ADDR : uint8_t {
  DRV8434S_REG_FAULT = 0x00,            // Fault status register.
  DRV8434S_REG_DIAG1 = 0x01,            // DIAG status 1.
  DRV8434S_REG_DIAG2 = 0x02,            // DIAG status 2.
  DRV8434S_REG_CTRL1 = 0x03,            // Control register 1.
  DRV8434S_REG_CTRL2 = 0x04,            // Control register 2.DRV8434S_FAULT
  DRV8434S_REG_CTRL3 = 0x05,            // Control register 3.
  DRV8434S_REG_CTRL4 = 0x06,            // Control register 4.
  DRV8434S_REG_CTRL5 = 0x07,            // Control register 5.
  DRV8434S_REG_CTRL6 = 0x08,            // Control register 6.
  DRV8434S_REG_CTRL7 = 0x09,            // Control register 7.
  DRV8434S_REG_CTRL8 = 0x0A,            // Control register 8.
  DRV8434S_REG_CTRL9 = 0x0B,            // Control register 9.
};


// FAULT REGISTER SETINGS ********************************************************************************************// 
enum DRV8434S_FAULT_Reg_Val : uint8_t {
  DRV8434S_FAULT_FAULT = 0x80,          // Fault bit is high when nFAULT output is low.
  DRV8434S_FAULT_SPI_ERR = 0x40,        // Indicates SPI protocol errors.
  DRV8434S_FAULT_UVLO = 0x20,           // Indicates undervoltage lockout fault condition.
  DRV8434S_FAULT_CPUV = 0x10,           // Indicates charge pump undervoltage fault condition.
  DRV8434S_FAULT_OCP = 0x08,            // Indicates overcurrent fault condition.
  DRV8434S_FAULT_STL = 0x04,            // Indicates motor stall condition.
  DRV8434S_FAULT_TF = 0x02,             // Indicates overtemperature warning/shutdown condition.
  DRV8434S_FAULT_OL = 0x01,             // Indicates open-load condition.
};


// DIAG STATUS 1 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_DIAG1_Reg_Val : uint8_t {
  DRV8434S_DIAG1_OCP_LS2_B = 0x80,      // Indicates overcurrent fault on the low-side FET of half bridge 2 in BOUT.
  DRV8434S_DIAG1_OCP_HS2_B = 0x40,      // Indicates overcurrent fault on the high-side FET of half bridge 2 in BOUT.
  DRV8434S_DIAG1_OCP_LS1_B = 0x20,      // Indicates overcurrent fault on the low-side FET of half bridge 1 in BOUT.
  DRV8434S_DIAG1_OCP_HS1_B = 0x10,      // Indicates overcurrent fault on the high-side FET of half bridge 1 in BOUT.
  DRV8434S_DIAG1_OCP_LS2_A = 0x08,      // Indicates overcurrent fault on the low-side FET of half bridge 2 in AOUT.
  DRV8434S_DIAG1_OCP_HS2_A = 0x04,      // Indicates overcurrent fault on the high-side FET of half bridge 2 in AOUT.
  DRV8434S_DIAG1_OCP_LS1_A = 0x02,      // Indicates overcurrent fault on the low-side FET of half bridge 1 in AOUT.
  DRV8434S_DIAG1_OCP_HS1_A = 0x01,      // Indicates overcurrent fault on the high-side FET of half bridge 1 in AOUT.
};


// DIAG STATUS 2 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_DIAG2_Reg_Val : uint8_t {
  DRV8434S_DIAG2_OTW = 0x40,            // Indicates overtemperature warning.
  DRV8434S_DIAG2_OTS = 0x20,            // Indicates overtemperature shutdown.
  DRV8434S_DIAG2_STL_LRN_OK = 0x10,     // Indicates stall detection learning is successful.
  DRV8434S_DIAG2_STALL = 0x08,          // Indicates motor stall condition.
  DRV8434S_DIAG2_OL_B = 0x02,           // Indicates open-load detection on BOUT.
  DRV8434S_DIAG2_OL_A = 0x01,           // Indicates open-load detection on AOUT
};


// CONTROL 1 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL1_Reg_Val : uint8_t {
  DRV8434S_CTRL1_TRQ_DAC = 0xF0,         // ??? (default is 0000b = 100%).
  DRV8434S_CTRL1_OL_MODE = 0x02,         // Open-load fault clearing mode.
};


// CONTROL 2 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL2_Reg_Val : uint8_t {
  DRV8434S_CTRL2_EN_OUT = 0x80,         // Write '0' to disable all outputs.
  DRV8434S_CTRL2_TOFF = 0x18,           // ??? (default is 01b = 16 us).
  DRV8434S_CTRL2_DECAY = 0x07,          // ??? (default is 111b = Smart tune Ripple Control).
};

enum DRV8434S_PWM_TOFF : uint8_t {
  DRV8434S_TOFF_7US   = 0b00,           // 7 μs.
  DRV8434S_TOFF_16US  = 0b01,           // 16 us.
  DRV8434S_TOFF_24US  = 0b10,           // 24 us.
  DRV8434S_TOFF_32US  = 0b11,           // 32 us.
};

enum DRV8434S_Decay_Mode : uint8_t {
  DRV8434_DECAY_SLOW_SLOW     = 0b000,  // Increasing SLOW, decreasing SLOW.
  DRV8434_DECAY_SLOW_MIX30    = 0b001,  // Increasing SLOW, decreasing MIXED 30%.
  DRV8434_DECAY_SLOW_MIX60    = 0b010,  // Increasing SLOW, decreasing MIXED 60%.
  DRV8434_DECAY_SLOW_FAST     = 0b011,  // Increasing SLOW, decreasing FAST.
  DRV8434_DECAY_MIX30_MIX30   = 0b100,  // Increasing MIXED 30%, decreasing MIXED 30%.
  DRV8434_DECAY_MIX60_MIX60   = 0b101,  // Increasing MIXED 60%, decreasing MIXED 60%.
  DRV8434_DECAY_DYNAMIC       = 0b110,  // Smart tune Dynamic Decay.
  DRV8434_DECAY_SMART_RIPPLE  = 0b111,  // Smart tune Ripple Control (default).
};


// CONTROL 3 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL3_Reg_Val : uint8_t {
  DRV8434S_CTRL3_DIR = 0x80,            // Direction input (when SPI_DIR = 1).
  DRV8434S_CTRL3_STEP = 0x40,           // Step input (when SPI_SETP = 1), logic '1' advances one step (self-clearing).
  DRV8434S_CTRL3_SPI_DIR = 0x20,        // Direction is controlled by SPI.
  DRV8434S_CTRL3_SPI_STEP = 0x10,       // Stepping is controlled by SPI.
  DRV8434S_CTRL3_MICROSTEP_MODE = 0x0F, // Micro-stepping mode (default is 0110b = 1/16 step).
};

enum DRV8434S_Micostep_Mode : uint8_t
{
  DRV8434S_MICROSTEP_1_100 = 0b0000,    // Full step (2-phase excitation) with 100% current.
  DRV8434S_MICROSTEP_1_71  = 0b0001,    // Full step (2-phase excitation) with 71% current.
  DRV8434S_MICROSTEP_2_NC  = 0b0010,    // Non-circular 1/2 step.
  DRV8434S_MICROSTEP_2     = 0b0011,    // 1/2 step.
  DRV8434S_MICROSTEP_4     = 0b0100,    // 1/4 step.
  DRV8434S_MICROSTEP_8     = 0b0101,    // 1/8 step.
  DRV8434S_MICROSTEP_16    = 0b0110,    // 1/16 step.
  DRV8434S_MICROSTEP_32    = 0b0111,    // 1/32 step.
  DRV8434S_MICROSTEP_64    = 0b1000,    // 1/64 step.
  DRV8434S_MICROSTEP_128   = 0b1001,    // 1/128 step.
  DRV8434S_MICROSTEP_256   = 0b1010,    // 1/256 step.
};


// CONTROL 4 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL4_Reg_Val : uint8_t {
  DRV8434S_CTRL4_CLR_FLT = 0x80,        // Clear all latched fault bits (self-resetting).
  DRV8434S_CTRL4_LOCK = 0x60,           // Lock registers and ignore further register writes except for CLR_FLT.
  DRV8434S_CTRL4_UNLOCK = 0x30,         // Unlock all registers.
  DRV8434S_CTRL4_EN_OL = 0x08,          // Enable open load detection.
  DRV8434S_CTRL4_OCP_MODE = 0x04,       // Overcurrent conditon fault latching (0 = latching, 1 = automatic retrying).
  DRV8434S_CTRL4_OTSD_MODE = 0x02,      // Overtemperature conditon fault latching (0 = latching, 1 = automatic recovery).
  DRV8434S_CTRL4_TW_REP = 0x01,         // Overtemperature warning reporting on nFAULT (0 = not reported, 1 = reported).
};


// CONTROL 5 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL5_Reg_Val : uint8_t {
  DRV8434S_CTRL5_STL_LRN = 0x20,        // Start the stall learning process (automatically resets when complete).
  DRV8434S_CTRL5_EN_STL = 0x10,         // Enable stall detection.
  DRV8434S_CTRL5_STL_REP = 0x08,        // Stall detection reporting on nFAULT (0 = not reported, 1 = reported).
};


// CONTROL 6 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL6_Reg_Val : uint8_t {
  DRV8434S_CTRL6_STALL_TH = 0xFF,       // Lower 8-btis of stall threshold (default is 00000011b).
};


// CONTROL 7 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL7_Reg_Val : uint8_t {
  DRV8434S_CTRL7_RC_RIPPLE = 0xC0,      // Ripple setting (00b = 1%, 01b = 2%, 10b = 4%, 11b = 6%).
  DRV8434S_CTRL7_EN_SSC = 0x20,         // Enable spread-spectrum (on by default).
  DRV8434S_CTRL7_TRQ_SCALE = 0x10,      // Apply 8x torque scaling (off by default).
  DRV8434S_CTRL7_STL_REP = 0x0F,        // Upper 4-bits of stall threshold.
};

enum DRV8434S_RC_Ripple : uint8_t {
  DRV8434S_RIPPLE_1 = 0b00,    // 1% ripple (default).
  DRV8434S_RIPPLE_2 = 0b01,    // 2% ripple (default).
  DRV8434S_RIPPLE_4 = 0b10,    // 4% ripple (default).
  DRV8434S_RIPPLE_6 = 0b11,    // 6% ripple (default).
};

// CONTROL 8 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL8_Reg_Val : uint8_t {
  DRV8434S_CTRL8_TRQ_COUNT = 0xFF,      // Lower 8-bits of TRQ_COUNT.
};


// CONTROL 9 REGISTER SETINGS ************************************************************************************// 
enum DRV8434S_CTRL9_Reg_Val : uint8_t {
  DRV8434S_CTRL9_REV_ID = 0xF0,         // Silicon revision identification.
  DRV8434S_CTRL9_TRQ_SCALE = 0x0F,      // Upper 4-bits of TRQ_COUNT.
};



#endif                                    // #ifndef DRV8434S_REGISTERS_H