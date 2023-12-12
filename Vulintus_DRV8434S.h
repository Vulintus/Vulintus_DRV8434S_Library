/*  Vulintus_DRV8434S.h - copyright Vulintus, Inc., 2023

    Vulintus STMicroelectronics DRV8434S eCompass module library.

    UPDATE LOG:
      2023-11-05 - Drew Sloan - Library first created.

*/


#ifndef VULINTUS_DRV8434S_H
#define VULINTUS_DRV8434S_H

#include <Arduino.h>						// Arduino main include file.
#include <SPI.h>							// Standard Arduino SPI library.

#include <Vulintus_Stepper_Driver.h>		// Vulintus wrapper library of common stepper functions.
#include <DRV8434S_Registers.h>				// DRV8434S register addresses and settings.


#define DRV8434S_SPI_SPEED	10000000		// SPI clock speed, in Hz.


//CLASSES ******************************************************************************************************//
class Vulintus_DRV8434S : public Vulintus_Stepper_Driver {

	public:
	
		// Error codes. //
		enum DRV8434S_errors : uint8_t {
			DRV8434S_NO_ERROR = 0,        		// Code to return indicating no error.
			DRV8434S_ERR_NO_FLT_PIN = 1,        // Fault pin is not set, can't monitor faults.
			DRV8434S_ERR_NO_SLP_PIN = 2,        // Sleep pin is not set, can't set sleep mode.
		};
		
		// Constructors. //
		Vulintus_DRV8434S(SPIClass *spi_bus, uint8_t pin_cs);		// SPI with chip-select.
		Vulintus_DRV8434S(SPIClass *spi_bus, uint8_t pin_cs, uint8_t pin_slp);		// SPI with chip-select and sleep input.

		// Variables. //
		volatile uint8_t fault;						// Fault code, used in interrupts.

		// Functions. //
		void begin(void); 							// Initialization.
		
		void set_fault_pin(uint8_t pin_flt);		// Set the fault pin.
		// uint8_t set_fault_interrupt(void);			// Set the fault interrupt (without specifying a pin).
		// void set_fault_interrupt(uint8_t pin_flt);	// Set the fault interrupt (specifying a pin).
		uint8_t read_fault(void);					// Read any active fault codes.
		void clear_faults(void);					// Clear all active faults.

		void reset(void);							// Reset all control registers to the default settings.
		bool verify(void);							// Verify the current control register settings.

		void set_vref_pin(uint8_t pin_vref);			// Set the current set reference pin.
		void set_vref_pwm(uint8_t pwm_val);				// Set the current set reference PWM value.
		void set_current_max(uint16_t max_milliamps);	// Set the maximum possible output current, in milliamps.
		void set_current(uint16_t milliamps);			// Set the target output current, in milliamps.
		uint16_t actual_current(void);					// Update the output current settings and return the actual current.

		void set_open_load_fault_latch(bool enable);		// Enable/disable open load fault latching (default on).
		void set_open_load_detection(bool enable);			// Enable/disable open load detection (default off).
		void set_pwm_toff(DRV8434S_PWM_TOFF toff);			// Set the PWM OFF time (default 16 us).
		void set_decay_mode(DRV8434S_Decay_Mode mode);		// Set the decay mode (default is smart tune ripple control).
		void set_overcurrent_fault_latch(bool enable);		// Enable/disable overcurrent fault latching (default on).
		void set_overtemperature_fault_latch(bool enable);	// Enable/disable overtemperature fault latching (default latching).
		void set_temperature_fault(bool enable);			// Enable/disable reporting of over/under temperature warnings on nFAULT (default off).
		void set_ripple_current(DRV8434S_RC_Ripple ripple);	// Set the ripple current (default is 19mA + 1% of I_trip).
		void set_spread_spectrum(bool enable);				// Enable/disable spread-spectrum (default on).

		void set_torque_scaling(bool enable);		// Enable/disable 8x torque count scaling (default off).
		uint16_t torque_count(void);				// Read the current torque count.
		
		void set_stall_detection(bool enable);		// Enable/disable stall detection (default is off).
		void learn_stall_count(void);				// Initiate stall count learning.
		void set_stall_fault(bool enable);			// Set reporting of stalls on nFAULT (default on).
		void set_stall_thresh(uint16_t thresh);		// Manually set the stall threshold (0-4095).
		uint16_t get_stall_thresh(void);			// Read the current stall threshold (0-4095).
		
		void set_dir_pin(uint8_t pin_dir);			// Set the direction pin (disables SPI direction-setting).
		void use_SPI_direction(bool enable);		// Use SPI for direction setting (default off).
		void set_direction(bool dir);				// Set the current direction.
		bool get_direction(void);					// Get the current direction.

		void set_step_pin(uint8_t pin_step);				// Set the step	pin (disables SPI stepping).
		void use_SPI_step(bool enable);						// Use SPI for stepping (default off).
		void step(void);									// Trigger one step.
		void set_microsteps(DRV8434S_Micostep_Mode mode);	// Set the microstep setting.

		void set_enable_pin(uint8_t pin_en);		// Set the enable pin (disables SPI enable).
		void use_SPI_enable(bool enable);			// Use SPI for enabling/disabling (default on).
		void enable(void);							// Enable the driver.
		void disable(void);							// Disable the driver

		void set_sleep_pin(uint8_t pin_slp);		// Set the sleep pin.		
		uint8_t sleep(bool enable);					// Enable/disable sleep mode.

		void set_lock(bool lock);			// Lock/unlock the registers (default is unlocked).
		uint8_t revision_id(void);			// Read the silicon revision ID.

		uint8_t write_register(uint8_t reg_addr, uint8_t reg_val); 	// Write to a register.
		uint8_t read_register(uint8_t reg_addr);  					// Read from a register.
		

	private:
		
		// Variables. //
		SPIClass *_spi_bus = NULL;				// SPI interface pointer.
		uint8_t _pin_cs;						// SPI chip select pin.
		
		uint8_t _pin_en, _pin_slp, _pin_flt;	// Enable, sleep, and fault pin.
		bool _hw_en, _hw_slp, _hw_flt; 			// Hardware/microcontroller connection flags for the enable, sleep, and fault pin.

		uint8_t _pin_dir, _pin_step; 			// Direction and step pins.

		uint8_t _pin_vref;						// Current set reference pin.
		uint8_t _vref_pwm = 255; 				// Current set reference PWM setting.

		uint8_t _ctrl_reg_val[7];				// Current control register 1-7 values.

		uint16_t _target_current = 500; 		// Target output current, in milliamps.
		uint16_t _max_current = 2500; 			// Maximum possible output current, in milliamps.

		// Functions. //
		void spi_start(void);					// Start an SPI transaction.
		void spi_end(void);						// End an SPI transaction.

		// void fault_interrupt();					// Interrupt function for the fault input.		

};

#endif 									//#ifndef VULINTUS_DRV8434S_H