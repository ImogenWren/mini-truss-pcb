/*--------------------------------------autoDelay Library Example--------------------------------

   Simple method for timing events without using interrupt based delay functions

    autoDelay != delay();

    functions include delays passed values in:
    Micro & Milliseconds,
    Seconds, and Minutes.
    
    Makes timing events a piece of cake.

    Written by Declan Heard. 09/08/2020
    
    Released into Public Domain
    Free to Distribute and Use.

*/




#include "autoDelay.h"                // Use " " Marks if library files are in local folder. 

//#include <autoDelay.h>              // Use < > Brackets if library files are in library folder.



autoDelay autodelay;    // set up autodelay instance.





// delay time variables:

int timerMillis = 500;

uint32_t timerMicros = 50000;

int timerSeconds = 2;

int timerMinutes = 1;





//---------Other Variables--- Only used for Example Sketch-----------------------------

int ledPin = 13;                                 // Set the Output Pin

int ledState = LOW;                            // state of the LED pin







void setup() {

  // autodelay.begin(9600);                          // begin function to set up serial communication. Defaults to 112500 Baud. Not required for correct function

  pinMode(ledPin, OUTPUT);                       // sets up LED pin as output pin

}





void loop() {





  if (autodelay.millisDelay(timerMillis)) {   // if this returns true, at once  // do the thing //



    //-----The Thing to be Done------

    if (ledState == HIGH) {

      ledState = LOW;

    } else if (ledState == LOW) {

      ledState = HIGH;
    }

    //-----^^^^^^^ End of The Thing thats done------




  }  // end of IF statement



  digitalWrite(ledPin, ledState);  // Write the LED state to the led pin.


}  // end of void loop





// ---- Alternative Timing Functions -----


//  if (autodelay.microsDelay(timerMicros)) {

//  if (autodelay.secondsDelay(timerSeconds)) {

//  if (autodelay.minutesDelay(timerMinutes)) {
