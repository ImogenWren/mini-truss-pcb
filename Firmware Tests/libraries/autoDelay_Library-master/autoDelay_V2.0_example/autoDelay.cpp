/*--------------------------------------autoDelay.h Library -------------------------------

   Simple method for timing events without using interrupt based delay functions

    autoDelay != delay();

    functions include delays passed values in:
    Micro & Milliseconds,
    Seconds, and Minuites.

    Makes timing events a piece of cake.

    Written by Declan Heard. 09/08/2020

    Released into Public Domain
    Free to Distibute and Use.

*/



#include "autoDelay.h"

autoDelay::autoDelay() {   // Anything needed to setup instance goes here

  // Serial.println("autoDelay Setup Instance");

}


void autoDelay::begin() {

  //	Serial.begin(baudrate);
  //	Serial.println("autoDelay Library deployed");


}


bool autoDelay::globalDelay(uint32_t currentTime, uint32_t delayTime) {     // Generic function uses generic time variable,
  // other functions just modify what value is passed for timing functions
  if (currentTime - previousTime	>= delayTime) {
    previousTime = currentTime;
    return true;
  } else {
    return false;
  }
}


bool autoDelay::delayEvent(uint32_t currentTime, uint32_t delayTime) {     // Loop function for delayEvents. Enacts any delayEvents set up by functions below.

  if (delayActive) {

    if  (autoDelay::globalDelay(currentTime, delayTime)) {

      if (eventCounter > 0) {                                         // eventCounter is decremented on event untill equal to zero
        eventCounter--;
        if (eventCounter == 0) {                                       // when eventCounter is zero
          delayActive = false;                                           // delay function is stopped
        }
      }
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}



void autoDelay::startDelay(int16_t repeats) {                          // Starts a delayEvent with (X No. Repeats). If passed negative argument will repeat indefinatly. Default -1
  delayActive = true;
  eventCounter = repeats;
}

void autoDelay::startIndefinate() {                                    // Starts delayEvent which repeats indefinatly
  delayActive = true;
  eventCounter = -1;
}

void autoDelay::stopDelay(bool resetRepeats) {                      // Stops a delayEvent with the option to reset eventCounter. defaults to true (pauseDelay is alternative which does not reset)
  delayActive = false;
  if (resetRepeats) {
    eventCounter = 0;
  }
}

void autoDelay::pauseDelay() {                                      // Pauses delayEvent and does not reset eventCounter
  delayActive = false;
}

void autoDelay::restartDelay(int16_t additional) {                 // Restarts delayEvent with the option to add in additional events. Defaults to 0
  delayActive = true;
  eventCounter = eventCounter + additional;
}

void autoDelay::addCounter(int16_t additional) {                     // Method to add additional events to delayEvent without pausing, or restarting delayEvent. Defaults to add 1
  eventCounter = eventCounter + additional;                          // Can be passed negative arguments to subtract events left.

  if (eventCounter < 0) {                                           // Safe function, if number of events subtracted are less than remaining events, defaults to 0,
    eventCounter = 0;                                                // else could trigger indefinate repeats. If indefiniate repeat is desired, use startDelay(-1) or startIndefinate();
  }
}                                                                   // Adding additional events to an indefinate delay event could lead to incorrect values. Best to use startDelay(X);






bool autoDelay::millisDelayEvent(uint32_t delayTime) {    // delayEvent function that can be passed Milliseconds

  uint32_t currentTime = millis();

  bool delayComplete = autoDelay::delayEvent(currentTime, delayTime);

  return delayComplete;

}



bool autoDelay::microsDelayEvent(uint32_t delayTime) {    // Delay function that can be passed Microseconds

  uint32_t  currentTime = micros();                             // N.B. micros has resolution of 4

  bool delayComplete = autoDelay::delayEvent(currentTime, delayTime);

  return delayComplete;

}



bool autoDelay::secondsDelayEvent(uint32_t delayTime) {    // Delay function that can be passed Seconds

  delayTime = delayTime * 1000;                     // multiply the passed seconds value by 1000 such that compared values are both in Milliseconds

  uint32_t currentTime = millis();                             //

  bool delayComplete = autoDelay::delayEvent(currentTime, delayTime);

  return delayComplete;

}


bool autoDelay::minutesDelayEvent(uint32_t delayTime) {    // Delay function that can be passed Minutes

  delayTime = delayTime * 60000;                     // multiply the passed minutes value by 60000 such that compared values are both in Millisseconds

  uint32_t  currentTime = millis();                            //

  bool delayComplete = autoDelay::delayEvent(currentTime, delayTime);

  return delayComplete;

}








bool autoDelay::millisDelay(uint32_t delayTime) {    // Delay function that can be passed Milliseconds

  uint32_t currentTime = millis();

  bool delayComplete = autoDelay::globalDelay(currentTime, delayTime);

  return delayComplete;

}



bool autoDelay::microsDelay(uint32_t delayTime) {    // Delay function that can be passed Microseconds

  uint32_t  currentTime = micros();	                           // N.B. micros has resolution of 4

  bool delayComplete = autoDelay::globalDelay(currentTime, delayTime);

  return delayComplete;

}



bool autoDelay::secondsDelay(uint32_t delayTime) {    // Delay function that can be passed Seconds

  delayTime = delayTime * 1000;                     // multiply the passed seconds value by 1000 such that compared values are both in Milliseconds

  uint32_t currentTime = millis();	                           //

  bool delayComplete = autoDelay::globalDelay(currentTime, delayTime);

  return delayComplete;

}


bool autoDelay::minutesDelay(uint32_t delayTime) {    // Delay function that can be passed Minutes

  delayTime = delayTime * 60000;                     // multiply the passed minutes value by 60000 such that compared values are both in Millisseconds

  uint32_t  currentTime = millis();	                           //

  bool delayComplete = autoDelay::globalDelay(currentTime, delayTime);

  return delayComplete;

}






// Depreciated // Old version

// This has been left in place to keep functionality with older versions of library
// Function has been changed to reflect new implmentation. Origional implementation is below for reference.



bool autoDelay::delayScript(uint32_t delayTime) {             // Passed DelayTime value
  // returns True if this time has passed since the last time it ran
  // remains true for a single loop

  uint32_t currentTime = millis();                  // sets a variable for the current milliseconds passed

  bool delayComplete = autoDelay::globalDelay(currentTime, delayTime);

  return delayComplete;

}  // end of delayTime routine







// Origional Implementation
/*

  bool autoDelay::delayScript(uint32_t delayTime) {             // Passed DelayTime value
  // returns True if this time has passed since the last time it ran
  // remains true for a single loop

  currentMillis = millis();                  // sets a variable for the current milliseconds passed


  if (currentMillis - previousMillis > delayTime) {  // if the delay time has been passed

    delayComplete = true;

    previousMillis = currentMillis;

  } else {

    delayComplete = false;

  }

  return delayComplete;

  }  // end of delayTime routine


*/
