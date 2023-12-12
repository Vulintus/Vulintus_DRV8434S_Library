/*  Vulintus_DRV8434S.cpp - copyright Vulintus, Inc., 2023

    Vulintus STMicroelectronics DRV8434S eCompass module library.

    UPDATE LOG:
      2023-03-21 - Drew Sloan - Library first created.

*/


#include <Vulintus_DRV8434S.h>          // Library header.


// CLASS PUBLIC FUNCTIONS ****************************************************// 


// Class constructor (SPI with chip-select).
Vulintus_DRV8434S::Vulintus_DRV8434S(SPIClass *spi_bus, uint8_t pin_cs)
    : _pin_cs(pin_cs)
{
  _spi_bus = spi_bus;                   // Set the SPI bus.
  pinMode(_pin_cs, OUTPUT);             // Set the CS pin mode to output.
  digitalWrite(_pin_cs, HIGH);          // Set the CS pin high.
}


// Class constructor (SPI with chip-select and sleep input).
Vulintus_DRV8434S::Vulintus_DRV8434S(SPIClass *spi_bus, uint8_t pin_cs, uint8_t pin_slp)
    : _pin_cs(pin_cs), _pin_slp(pin_slp)
{
  _spi_bus = spi_bus;                   // Set the SPI bus.
  pinMode(_pin_cs, OUTPUT);             // Set the CS pin mode to output.
  digitalWrite(_pin_cs, HIGH);          // Set the CS pin high.
  set_sleep_pin(_pin_slp);              // Set the sleep pin.
}


// Initialization.
void Vulintus_DRV8434S::begin(void)
{
    _spi_bus->begin();                  // Initialize the SPI bus.
    reset();                            // Reset all registers to the default settings.
}


// Set the fault pin.
void Vulintus_DRV8434S::set_fault_pin(uint8_t pin_flt)
{
  _pin_flt = pin_flt;                   // Set the fault pin.
  pinMode(_pin_flt, INPUT);             // Set the fault pin mode to INPUT.
  _hw_flt = 1;                          // Set the hardware fault flag to 1.
}


// // Set the fault interrupt (without specifying a pin).
// uint8_t Vulintus_DRV8434S::set_fault_interrupt(void)
// {
//   if (_hw_flt) {                                          // If a fault pin is set...
//     attachInterrupt(digitalPinToInterrupt(_pin_flt), 
//           fault_interrupt, FALLING);                      // Set a falling interrupt.
//     return DRV8434S_NO_ERROR;                             // Return the no-error code.
//   }
//   else {                                                  // Otherwise, if no fault pin is set...
//     return DRV8434S_ERR_NO_FLT_PIN;                       // Return an error.
//   }
// }


// // Set the fault interrupt (specifying a pin).
// void Vulintus_DRV8434S::set_fault_interrupt(uint8_t pin_flt)
// {
//   Vulintus_DRV8434S::set_fault_pin(pin_flt);              // Set the fault pin.
//   attachInterrupt(digitalPinToInterrupt(_pin_flt), 
//         fault_interrupt, FALLING);                        // Set a falling interrupt.
// }


// Read any active fault codes.
uint8_t Vulintus_DRV8434S::read_fault(void)
{
  fault = read_register(DRV8434S_REG_FAULT);    //Read the value of the fault register.
  return fault;                                 //Return the fault value.
}


// Clear all active faults.
void Vulintus_DRV8434S::clear_faults(void)
{
  _ctrl_reg_val[3] != DRV8434S_CTRL4_CLR_FLT;             // Set the clear fault bit.
  write_register(DRV8434S_REG_CTRL4, _ctrl_reg_val[3]);   // Update control register 4.
}		


// Reset all registers to the default settings.
void Vulintus_DRV8434S::reset(void)
{
  // Starting control register values (differs slightly from defaults).
  _ctrl_reg_val[0] = 0x00;       // TRQ_DAC = 100%, nFAULT released after latched OL fault by CLR_FLT or nSLEEP pulse.
  _ctrl_reg_val[1] = 0x0F;       // Outputs disabled, TOFF = 16 us, smart tune ripple control.
  _ctrl_reg_val[2] = 0x30;       // DIR = 0, SPI direction control, SPI step control, full step with 100% current mode.
  _ctrl_reg_val[3] = 0x30;       // Unlocked, open load detection disabled, overcurrent and/or overtemperature latches fault, no temperature warnigns on nFAULT.
  _ctrl_reg_val[4] = 0x08;       // Not learning stall count, stall detection disabled, stall detection reported on nFAULT.
  _ctrl_reg_val[5] = 0x03;       // Stall threshold = 3 counts.
  _ctrl_reg_val[6] = 0x20;       // 1% ripple, spread-spectrum enabled, no torque count scaling, stall threshold = 3 counds.

  write_register(DRV8434S_REG_CTRL1, _ctrl_reg_val[0]);   // Control register 1.
  write_register(DRV8434S_REG_CTRL2, _ctrl_reg_val[1]);   // Control register 2.
  write_register(DRV8434S_REG_CTRL3, _ctrl_reg_val[2]);   // Control register 3.
  write_register(DRV8434S_REG_CTRL4, _ctrl_reg_val[3]);   // Control register 4.
  write_register(DRV8434S_REG_CTRL5, _ctrl_reg_val[4]);   // Control register 5.
  write_register(DRV8434S_REG_CTRL6, _ctrl_reg_val[5]);   // Control register 6.
  write_register(DRV8434S_REG_CTRL7, _ctrl_reg_val[6]);   // Control register 7.
}


// Verify the current control register settings.
bool Vulintus_DRV8434S::verify(void)
{
  uint8_t ret = read_register(DRV8434S_REG_CTRL1) == _ctrl_reg_val[0] &&
                read_register(DRV8434S_REG_CTRL2) == _ctrl_reg_val[1] &&
                read_register(DRV8434S_REG_CTRL3) == _ctrl_reg_val[2] &&
                read_register(DRV8434S_REG_CTRL4) == _ctrl_reg_val[3] &&
                read_register(DRV8434S_REG_CTRL5) == _ctrl_reg_val[4] &&
                read_register(DRV8434S_REG_CTRL6) == _ctrl_reg_val[5] &&
                read_register(DRV8434S_REG_CTRL7) == _ctrl_reg_val[6];
  return ret;
}	


// Set the current set reference pin.
void Vulintus_DRV8434S::set_vref_pin(uint8_t pin_vref)
{
  _pin_vref = pin_vref;                  // Set the current set reference pin.
  pinMode(_pin_vref, OUTPUT);            // Set the current set reference pin mode to INPUT.
  set_vref_pwm(_vref_pwm);               // Set the current set reference PWM output.
}


// Set the current set reference PWM value.
void Vulintus_DRV8434S::set_vref_pwm(uint8_t pwm_val)
{
  _vref_pwm = pwm_val;                            // Save the PWM value setting for current calculations.
  if ((_vref_pwm == 255) || (_vref_pwm == 0)) {   // If the PWM value is at maximum or minimum...
    digitalWrite(_pin_vref, _vref_pwm);           // Use digitalWrite to set the PWM (in case it's not actual a PWM pin).
  }
  else {                                          // For all other PWM values...
    analogWrite(_pin_vref, _vref_pwm);            // Use analogWrite.
  }
  actual_current();                               //Update the current settings.
}


// Set the maximum possible full-scale output current, in milliamps.
void Vulintus_DRV8434S::set_current_max(uint16_t max_milliamps)
{
  if (max_milliamps > 2500) {           // If the setting exceeds the driver's maximum...
    _max_current = 2500;                // Set the maximum possible output current to 2500 mA.
  }
  else {                                // Otherwise...
    _max_current = max_milliamps;       // Set the maximum possible output current to the specified value.
  }
	actual_current();                     // Update the current settings.
}


// Set the full-scale output current, in milliamps.
void Vulintus_DRV8434S::set_current(uint16_t milliamps)
{
  if (milliamps > 2500) {               // If the specified value is greater than  the driver's maximum...
    _target_current = 2500;             // Set the target current to to 2500 mA.
  } 
  else {                                // Otherwise...
    _target_current = milliamps;        // Set the target current to the specified value.
  }
  actual_current();                     // Update the current settings.
}


// Update the output current settings and return the actual current.
uint16_t Vulintus_DRV8434S::actual_current(void) {
  uint16_t max_range = (uint32_t) _vref_pwm * _max_current / 255;               // Find the maximum possible current.
  uint8_t trq_dac_setting = (uint16_t) (16 * _target_current) / max_range;      // Find the closest 1/16th step.
  uint16_t actual;                                        // Actual current setting.

  if (trq_dac_setting == 0) {                             // If the calculated setting was 0...
    trq_dac_setting = 1;                                  // Set the setting to 1.
  }
  else if (trq_dac_setting > 16) {                        // If the calculated setting exceeds 16...
    trq_dac_setting = 16;                                 // Set the setting to 16.
  }
  actual = (trq_dac_setting * max_range) / 16;            // Calculate the actual current limit.
  trq_dac_setting = 16 - trq_dac_setting;                 // Flip the TRQ_DAC setting.

  _ctrl_reg_val[0] &= 0x0F;                               // Clear out the current TRQ_DAC bits in the CTRL1 register value.
  _ctrl_reg_val[0] |= (trq_dac_setting << 4);             // Set the new TRQ_DAC bits.
  write_register(DRV8434S_REG_CTRL1, _ctrl_reg_val[0]);   // Update control register 1.

  return actual;                                          // Return the actual current limit.
}


// void Vulintus_DRV8434S::set_open_load_fault_latch(bool enable);		// Enable/disable open load fault latching (default on).
// void Vulintus_DRV8434S::set_open_load_detection(bool enable);			// Enable/disable open load detection (default off).
// void Vulintus_DRV8434S::set_pwm_toff(DRV8434S_PWM_TOFF toff);			// Set the PWM OFF time (default 16 us).
// void Vulintus_DRV8434S::set_decay_mode(DRV8434S_Decay_Mode mode);		// Set the decay mode (default is smart tune ripple control).
// void Vulintus_DRV8434S::set_overcurrent_fault_latch(bool enable);		// Enable/disable overcurrent fault latching (default on).
// void Vulintus_DRV8434S::set_overtemperature_fault_latch(bool enable);	// Enable/disable overtemperature fault latching (default latching).
// void Vulintus_DRV8434S::set_temperature_fault(bool enable);			// Enable/disable reporting of over/under temperature warnings on nFAULT (default off).
// void Vulintus_DRV8434S::set_ripple_current(DRV8434S_RC_Ripple ripple);	// Set the ripple current (default is 19mA + 1% of I_trip).
// void Vulintus_DRV8434S::set_spread_spectrum(bool enable);				// Enable/disable spread-spectrum (default on).

// void Vulintus_DRV8434S::set_torque_scaling(bool enable);		// Enable/disable 8x torque count scaling (default off).
// uint16_t Vulintus_DRV8434S::torque_count(void);				// Read the current torque count.

// void Vulintus_DRV8434S::set_stall_detection(bool enable);		// Enable/disable stall detection (default is off).
// void Vulintus_DRV8434S::learn_stall_count(void);				// Initiate stall count learning.
// void Vulintus_DRV8434S::set_stall_fault(bool enable);			// Set reporting of stalls on nFAULT (default on).
// void Vulintus_DRV8434S::set_stall_thresh(uint16_t thresh);		// Manually set the stall threshold (0-4095).
// uint16_t Vulintus_DRV8434S::get_stall_thresh(void);			// Read the current stall threshold (0-4095).


// Set the direction pin (disables SPI direction-setting).
void Vulintus_DRV8434S::set_dir_pin(uint8_t pin_dir)
{
  _pin_dir = pin_dir;                   // Set the direction pin private variable.
  pinMode(_pin_dir, OUTPUT);            // Set the direction pin mode to OUTPUT.
  digitalWrite(_pin_dir, LOW);          // Set the direction pin low.
  use_SPI_direction(0);                 // Disable SPI direction control.
};			


// Use SPI for direction setting (default off).
void Vulintus_DRV8434S::use_SPI_direction(bool enable)
{
  if (enable) {                                           // If we're switching to SPI direction control...
    _ctrl_reg_val[2] |= DRV8434S_CTRL3_SPI_DIR;           // Set the DIR step control bit high in control register 3.
  }
  else {                                                  // If we're switching to hardware direction control...
    _ctrl_reg_val[2] &= ~DRV8434S_CTRL3_SPI_DIR;          // Set the DIR step control bit low in control register 3.    
  }
  write_register(DRV8434S_REG_CTRL3, _ctrl_reg_val[2]);   // Update control register 3.
}


// Set the direction with SPI.
void Vulintus_DRV8434S::set_direction(bool dir) {
  if (_ctrl_reg_val[2] & DRV8434S_CTRL3_SPI_DIR) {          // If we're using SPI for direction setting...
    _ctrl_reg_val[2] &= ~DRV8434S_CTRL3_DIR;                // Clear the current direction bit.    
    _ctrl_reg_val[2] |= (dir << 7);                         // Set the DIR bit to the specified high/low value in control register 3.
    write_register(DRV8434S_REG_CTRL3, _ctrl_reg_val[2]);   // Update control register 3.
  }
  else {                                                    // Otherwise, if we're using the DIR input for direction...
    digitalWrite(_pin_dir, dir);                            // Set the step pin to the specified high/low value.
  }
}


// Get the current direction.
bool Vulintus_DRV8434S::get_direction(void) 
{
  bool dir;                                         // Boolean direction.
  if (_ctrl_reg_val[2] & DRV8434S_CTRL3_SPI_DIR) {  // If we're using SPI for direction setting...
    dir = _ctrl_reg_val[2] & DRV8434S_CTRL3_DIR;    // Grab the value of the DIR bit in control register 3.
  }
  else {                                            // Otherwise, if we're using the DIR input for direction...
    dir = digitalRead(_pin_dir);                    // Read the current state of the hardware direction pin.
  }
  return dir;                                       // Return the direction.
}					


// Set the step	pin (disables SPI stepping).
void Vulintus_DRV8434S::set_step_pin(uint8_t pin_step)
{
  _pin_step = pin_step;                 // Set the step pin private variable.
  pinMode(_pin_step, OUTPUT);           // Set the step pin mode to OUTPUT.
  digitalWrite(_pin_step, LOW);         // Set the step pin low.
  use_SPI_step(0);                      // Disable SPI stepping.
}			


// Use SPI for stepping (default off).
void Vulintus_DRV8434S::use_SPI_step(bool enable)
{
  if (enable) {                                                 // If we're switching to SPI step control...
    _ctrl_reg_val[2] |= DRV8434S_CTRL3_SPI_STEP;                // Set the SPI step control bit high in control register 3.
  }
  else {                                                        // If we're switching to hardware step control...
    _ctrl_reg_val[2] &= ~DRV8434S_CTRL3_SPI_STEP;               // Set the SPI step control bit low in control register 3.    
  }
  write_register(DRV8434S_REG_CTRL3, _ctrl_reg_val[2]);         // Update control register 3.
}	


// Trigger a step.
void Vulintus_DRV8434S::step(void)
{
  if (_ctrl_reg_val[2] & DRV8434S_CTRL3_SPI_STEP) {             // If we're using SPI for stepping...
    _ctrl_reg_val[2] |= DRV8434S_CTRL3_STEP;                    // Set the STEP bit high in control register 3.
    write_register(DRV8434S_REG_CTRL3, _ctrl_reg_val[2]);       // Update control register 3.
  }
  else {                                                        // Otherwise, if we're using the STEP input for stepping...
    digitalWrite(_pin_step, HIGH);                              // Set the step pin high.
    delayMicroseconds(1);                                       // Pause for 1 microsecond.
    digitalWrite(_pin_step, LOW);                               // Set the step pin low.
  }
}		


// Set the microstep setting.
void Vulintus_DRV8434S::set_microsteps(DRV8434S_Micostep_Mode mode)
{
  _ctrl_reg_val[2] &= ~DRV8434S_CTRL3_MICROSTEP_MODE;     // Clear the microstep setting in control register 3.
  _ctrl_reg_val[2] |= mode;                               // Set the specified microstep mode.
  write_register(DRV8434S_REG_CTRL3, _ctrl_reg_val[2]);   // Update control register 3.
}


// Set the enable pin.
void Vulintus_DRV8434S::set_enable_pin(uint8_t pin_en)
{
  _pin_en = pin_en;                                       // Set the enable pin.
  pinMode(_pin_en, OUTPUT);                               // Set the enable pin mode to OUTPUT.
  digitalWrite(_pin_en, LOW);                             // Set the enable pin low (driver disabled).
  _hw_en = 1;                                             // Set the hardware enable flag to 1.
  _ctrl_reg_val[1] |= DRV8434S_CTRL2_EN_OUT;              // Set the enable bit high in control register 2.
  write_register(DRV8434S_REG_CTRL2, _ctrl_reg_val[1]);   // Update control register 2.
}


// Use SPI for enabling/disabling (default on).
void Vulintus_DRV8434S::use_SPI_enable(bool enable)
{
  if ((enable) && (_hw_en)) {                               // If we're switching to SPI for enabling/disabling...
    _ctrl_reg_val[1] &= ~DRV8434S_CTRL2_EN_OUT;             // Set the enable bit low in control register 2.
    write_register(DRV8434S_REG_CTRL2, _ctrl_reg_val[1]);   // Update control register 2.
    digitalWrite(_pin_en, HIGH);                            // Set the hardware enable pin high.
    _hw_en = 0;                                             // Set the hardware enable flag low.
  }
  else {                                                    // Otherwise...
    digitalWrite(_pin_en, LOW);                             // Set the hardware enable pin low.
    _hw_en = 1;                                             // Set the hardware enable flag high.
  }
}


// Enable the driver.
void Vulintus_DRV8434S::enable(void)
{
  if (!(_ctrl_reg_val[1] & DRV8434S_CTRL2_EN_OUT)) {        // If the EN_OUT bit isn't already set high...
    _ctrl_reg_val[1] |= DRV8434S_CTRL2_EN_OUT;              // Set the enable bit high in control register 2.
    write_register(DRV8434S_REG_CTRL2, _ctrl_reg_val[1]);   // Update control register 2.
  }
  if (_hw_en) {                                             // If the hardware enable line is connected...
    digitalWrite(_pin_en, HIGH);                            // Set the enable pin high.
  }
}


// Disable the driver.
void Vulintus_DRV8434S::disable(void)
{
  if (_hw_en) {                                             // If the hardware enable line is connected...
    digitalWrite(_pin_en, LOW);                             // Set the enable pin low.
  }
  else {                                                    // Otherwise...
    _ctrl_reg_val[1] &= ~DRV8434S_CTRL2_EN_OUT;             // Set the enable bit low in control register 2.
    write_register(DRV8434S_REG_CTRL2, _ctrl_reg_val[1]);   // Update control register 2.
  }
}	


// Set the sleep pin.
void Vulintus_DRV8434S::set_sleep_pin(uint8_t pin_slp)
{
  _pin_slp = pin_slp;                   // Set the sleep pin.
  pinMode(_pin_slp, OUTPUT);            // Set the sleep pin mode to OUTPUT.
  digitalWrite(_pin_slp, HIGH);         // Set the sleep pin high (sleep disabled).
  _hw_slp = 1;                          // Set the hardware sleep flag to 1.
}


// Enable/disable sleep mode.
uint8_t Vulintus_DRV8434S::sleep(bool enable) {
  if (_hw_slp) {                        // If the sleep line is connected...
    digitalWrite(_pin_slp, !enable);    // Set the sleep line to the specified value.
    return DRV8434S_NO_ERROR;           // Return a 1 to indicate success.
  }
  else {                                // Otherwise, if the sleep input wasn't set...
    return DRV8434S_ERR_NO_SLP_PIN;     // Return an error.
  }
}


// Lock/unlock the registers (default is unlocked).
void Vulintus_DRV8434S::set_lock(bool lock)
{
  _ctrl_reg_val[3] &= 0x8F;                     // Clear the lock bits in control register 4.
  if (lock) {                                   // If we're locking the registers...
    _ctrl_reg_val[3] != DRV8434S_CTRL4_LOCK;    // Set the lock bits.
  }
  else {                                        // Otherwise, if we're unlocking the registers...
    _ctrl_reg_val[3] != DRV8434S_CTRL4_UNLOCK;  // Set the unlock bits.
  }
  write_register(DRV8434S_REG_CTRL4, _ctrl_reg_val[3]);   // Update control register 4.
}


// Read the silicon revision ID.
uint8_t Vulintus_DRV8434S::revision_id(void)
{
  uint8_t ret = read_register(DRV8434S_REG_CTRL9);      // Read in the current value of control register 9.
  ret = ret >> 4;                                       // Shift over bits 4-7.
  return ret;                                           // Return the 4-bit number.
}		


// Write to a register.
uint8_t Vulintus_DRV8434S::write_register(uint8_t reg_addr, uint8_t new_data)
{
  uint8_t cmd = (reg_addr << 1);            // A write command has a zero on bit 7.
	uint8_t old_data;			                    // Existing data in register.

  spi_start();                              // Start the SPI transaction.
  uint8_t status = _spi_bus->transfer(cmd); // Send the command/address byte.
  old_data = _spi_bus->transfer(new_data);  // Send the new data/register value.
  spi_end();                                // End the SPI transaction.

  fault |= (status & 0x3F);                 // Update the fault code if any fault bits were high.

	return old_data;											    // Return the pre-exisitng register data.
}


// Read from a register.
uint8_t Vulintus_DRV8434S::read_register(uint8_t reg_addr)
{
  uint8_t cmd = 0x40 | (reg_addr << 1);     // A read command has a one on bit 7.
	uint8_t data;			                        // Current register data.

  spi_start();                              // Start the SPI transaction.
  uint8_t status = _spi_bus->transfer(cmd); // Send the command/address byte.
  data = _spi_bus->transfer(0);             // Send the a dummy byte to get a byte back.
  spi_end();                                // End the SPI transaction.

  fault |= (status & 0x3F);                 // Update the fault code if any fault bits were high.

	return data;											        // Return the pre-exisitng register data.
}


// CLASS PRIVATE FUNCTIONS ***************************************************// 

// Start an SPI transaction.
void Vulintus_DRV8434S::spi_start(void)
{
  digitalWrite(_pin_cs, LOW);     // Set the chip select line low.
  _spi_bus->beginTransaction(SPISettings(DRV8434S_SPI_SPEED, MSBFIRST, SPI_MODE1));   // Set the SPI settings for this chip.  
  delayMicroseconds(1);           // Pause for 1 microsecond. 
}


// End an SPI transaction.
void Vulintus_DRV8434S::spi_end(void)
{
  _spi_bus->endTransaction();     // Release the SPI bus.
  digitalWrite(_pin_cs, HIGH);    // Set the chip select line high. 
}


// // Interrupt function for the fault input.
// void Vulintus_DRV8434S::fault_interrupt()
// {
//   drv8434s_fault |= DRV8434S_FAULT_FAULT;   //Set the fault flag to 1.
// }	