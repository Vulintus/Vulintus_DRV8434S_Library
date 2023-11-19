/*  Vulintus_DRV8434S_Pin_Controll.ino

    copyright Vulintus, Inc., 2023

    Hardware pin-controlled stepping example for the Vulintus DRV8434S stepper 
    driver library.

    UPDATE LOG:
      2023-11-06 - Drew Sloan - Example first created.

*/


// Included libraries. //
#include <Vulintus_DRV8434S.h>          // Vulintus DRV8434S library.

// Serial communication constants. //
#define SERIAL_BAUD_RATE  115200        //Serial baud rate.

//Pin definitions.
#define PIN_DRV_EN          5
#define PIN_DRV_SLP         4
#define PIN_DRV_DIR         6
#define PIN_DRV_STEP        7
#define PIN_DRV_CS          8
#define PIN_DRV_FLT         3

// Stepper driver. //
// Vulintus_DRV8434S stepper = Vulintus_DRV8434S(&SPI, PIN_DRV_CS);             // << Call this if SLP is externally pulled up.
Vulintus_DRV8434S stepper = Vulintus_DRV8434S(&SPI, PIN_DRV_CS, PIN_DRV_SLP);   // << Call this if SLP is NOT externally pulled up.
const uint16_t start_step_period = 750;     //Starting step period, in microseconds.
const uint16_t min_step_period = 500;       //Minimum step period (at max. speed), in microseconds.
uint16_t cur_step_period;                   //Current step period, decreases as speed ramps up.
const uint16_t num_steps = 6500;            //Number of steps/microsteps to move.
bool dir;                                   //Boolean for flipping the direction.


// INITIALIZATION ************************************************************//
void setup() {

  //Initialize serial data transmission.
  Serial.begin(SERIAL_BAUD_RATE);        

  //Print the filename, date, and time to verify the upload.         
  Print_FW_Filename();
  Print_FW_Date(); 
  Print_FW_Time(); 
  Serial.println();

  //Initialize the DRV8434S (Starts SPI and resets all registers).
  stepper.begin();

  //Set the enable, step, and direction pins.
  stepper.set_enable_pin(PIN_DRV_EN);
  stepper.set_dir_pin(PIN_DRV_DIR);
  stepper.set_step_pin(PIN_DRV_STEP);

  //Set the output current.
  stepper.set_current_max(1000);        //Maximum possible current based on the maximum voltage on the VREF pin (Imax = VREF / 1.32 V/A).
  stepper.set_current(500);             //Target output current.

  //Check the actual output current after setting the 4-bt TRQ_DAC.
  Serial.print("stepper.actual_current = ");
  Serial.print(stepper.actual_current());
  Serial.println(" mA");

  //Verify all registers are correctly set.
  Serial.print("\nstepper.verify() = ");    
  Serial.println(stepper.verify()); 

  //Clear any active faults.
  stepper.clear_faults();     
}


// MAIN LOOP *****************************************************************//
void loop() {

  dir = !dir;                                                 // Flip the direction.
  Serial.print("\ndirection = ");                             // Show the current direction.
  Serial.println(dir);                                        // Show the current direction.
  stepper.set_direction(dir);                                 // Set the direction.
  stepper.enable();						                                // Enable the driver.
  cur_step_period = start_step_period;                        // Set the starting step period.
  for (uint16_t i = 0; i < num_steps; i++) {                  // Iterate through a discrete number of steps.
    stepper.step();                                           // Trigger a step.
    if (cur_step_period > min_step_period) {                  // If the current step period is greater than the minimum...
      cur_step_period--;                                      // Decrement the step period.
    };
    delayMicroseconds(cur_step_period);                       // Pause for the step period.
  }
  stepper.disable();							                            // Disable the driver.

  delay(1000);                                                // Pause for 1 second.

}


// SUBFUNCTIONS **************************************************************//

//Print the sketch filename from the macro.
void Print_FW_Filename(void) {
  char s[] = __FILE__;                  //Grab the filename from the macro.
  uint8_t b = sizeof(s);                //Find the number of characters in the filename.
  while ((b > 0) && (s[b] != '\\')) {   //Loop until a forward slash is found.
    b--;                                //Step backward through the filename.
  }   
  char *u = &s[++b];                    //Create a pointer to start at the filename.
  Serial.println(u);                    //Print the filename.
}


//Print the sketch upload date from the macro.
void Print_FW_Date(void) {
  char s[] = __DATE__;                  //Grab the upload date from the macro.
  Serial.println(s);                    //Print the upload date.
}


//Print the sketch upload time from the macro.
void Print_FW_Time(void) {
  char s[] = __TIME__;                  //Grab the upload time from the macro.
  Serial.println(s);                    //Print the upload date.
}