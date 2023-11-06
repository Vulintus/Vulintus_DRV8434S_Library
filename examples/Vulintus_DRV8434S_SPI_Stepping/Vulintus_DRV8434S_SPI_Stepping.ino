/*  Vulintus_DRV8434S_SPI_Stepping.ino

    copyright Vulintus, Inc., 2023

    SPI-controlled stepping example for the Vulintus DRV8434S stepper driver
    library.

    UPDATE LOG:
      2023-11-05 - Drew Sloan - Example first created.

*/


//Included libraries.//
#include <Vulintus_DRV8434S.h>           // Vulintus DRV8434S library.


//Serial communication constants.//
#define SERIAL_BAUD_RATE  115200        //Serial baud rate.

//Pin definitions.
#define PIN_DRV_EN          5
#define PIN_DRV_SLP         4
#define PIN_DRV_DIR         6
#define PIN_DRV_STEP        7
#define PIN_DRV_CS          8
#define PIN_DRV_FLT         3

//Stepper driver object.
Vulintus_DRV8434S stepper = Vulintus_DRV8434S(&SPI, PIN_DRV_CS);


// INITIALIZATION ************************************************************//
void setup() {

  //Initialize serial data transmission.
  Serial.begin(SERIAL_BAUD_RATE);                            
  Print_FW_Filename();          
  Print_FW_Date();

  //Initialize the DRV8434S 
  stepper.begin();                        //Starts SPI and resets all registers.


  stepper.set_fault_pin(PIN_DRV_FLT);   //Set the fault reporting input.

  //Set the output current.
  stepper.set_current_max(1000);        //Maximum possible current based on the maximum voltage on the VREF pin (Imax = VREF / 1.32 V/A).
  stepper.set_current(400);             //Target output current.

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
}


// SUBFUNCTIONS **************************************************************//

//Print the sketch filename from the macro.
void Print_FW_Filename() {
  char s[] = __FILE__;                    //Grab the filename from the macro.
  uint8_t b = sizeof(s);                  //Find the number of characters in the filename.
  while ((b > 0) && (s[b] != '\\')) {     //Loop until a forward slash is found.
    b--;                                  //Step backward through the filename.
  }   
  char *u = &s[++b];                      //Create a pointer to start at the filename.
  Serial.println(u);                      //Print the filename.
}


//Print the sketch upload date from the macro.
void Print_FW_Date() {
  char s[] = __DATE__;                    //Grab the upload date from the macro.
  Serial.println(s);                      //Print the upload date.
}


//Print the sketch upload time from the macro.
void Print_FW_Time() {
  char s[] = __TIME__;                  //Grab the upload time from the macro.
  Serial.println(s);                      //Print the upload date.
}