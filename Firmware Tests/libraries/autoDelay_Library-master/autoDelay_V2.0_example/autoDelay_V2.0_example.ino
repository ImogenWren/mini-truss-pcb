/*--------------------------------------autoDelay Library Example V2.0--------------------------------

   Simple method for timing events without using interrupt based delay functions

    autoDelay != delay();

    functions include delays passed values in:
    Micro & Milliseconds,
    Seconds, and Minuites.

    Makes timing events a piece of cake.

    Updated by Wren. 28/12/2020

    Released into Public Domain
    Free to Distibute and Use.

    /////// Updated V2.0 //////////

    Now includes methods for delayEvents, delay functions that 
    repeat events a certain number of times, and can be paused,
    restarted and updated.

*/



//#include <autoDelay.h>              // Use < > Brackets if library files are in library folder.

#include "autoDelay.h"

autoDelay delayOne;    // set up autodelay instance.
autoDelay delayTwo;



#define DELAY_ONE 1000     // Delay Constants
#define DELAY_TWO 170


#define LED_BLUE 5   // Define LED pins
#define LED_RED 3    // Define LED pins

bool ledBlue_state = false;             // Initial LED states
bool ledRed_state = false;


void setup() {

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_BLUE, ledBlue_state);
  digitalWrite(LED_RED, ledRed_state);
  delayOne.startDelay(-1);                  // startDelay function makes delayEvent active with (argument) number of events
}


void loop() {
  if (delayOne.millisDelayEvent(DELAY_ONE)) {    // Delay event must be triggered by startDelay()
    ledBlue_state = toggleOutput(LED_BLUE, ledBlue_state);
  }
  if (delayTwo.millisDelay(DELAY_TWO)) {   // Delay repeats indefinatly
    ledRed_state = toggleOutput(LED_RED, ledRed_state);
  }

}




bool toggleOutput(int outputPin, bool currentState) {
  if (currentState == true) {                     // Toggle the Output
    digitalWrite(outputPin, LOW);
    return false;
  } else {
    digitalWrite(outputPin, HIGH);
    return true;                                    // Return the new state
  }
}


/*


    if (delayOne.millisDelay(DELAY_ONE)) {   // if this returns true, at once  // do the thing //
    ledRed_state = toggleOutput(LED_RED, ledRed_state);
  }
*/
