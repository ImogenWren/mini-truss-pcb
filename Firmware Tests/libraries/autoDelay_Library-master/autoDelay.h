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


#ifndef autoDelay_h
#define autoDelay_h

#if (ARDUINO >=100)
#include <Arduino.h>
#else
#include <wProgram.h>
#endif


class autoDelay
{
  public:

    // Constructor
    autoDelay();

    // Methods


    void begin();              // depreciated to use with ATtiny


    // Basic Methods (Origional - depreciated but remain for backwards compatability)

    bool millisDelay(uint32_t delayTime);           // delay function using Milliseconds

    bool microsDelay(uint32_t delayTime);           // delay function using Microseconds

    bool secondsDelay(uint32_t delayTime);           // delay function using Seconds

    bool minutesDelay(uint32_t delayTime);           // delay function using Minutes


    // delayEvent Loop Methods

    bool millisDelayEvent(uint32_t delayTime);           // delay function using Milliseconds

    bool microsDelayEvent(uint32_t delayTime);           // delay function using Microseconds

    bool secondsDelayEvent(uint32_t delayTime);           // delay function using Seconds

    bool minutesDelayEvent(uint32_t delayTime);           // delay function using Minutes

    // delayEvent Control Methods

    void startDelay(int16_t repeats = -1);

    void startIndefinate();

    void stopDelay(bool resetRepeats = true);                   // Stops a delayEvent with the option to reset eventCounter. defaults to true



    void pauseDelay();                                    // Pauses delayEvent and does not reset eventCounter


    void restartDelay(int16_t additional = 0);                 // Restarts delayEvent with the option to add in additional events. Defaults to 0


    void addCounter(int16_t additional = 1);                    // Method to add additional events to delayEvent without pausing, or restarting delayEvent. Default adds 1
    // Can be passed negative arguments to subtract events left.
    // Safe function, if number of events subtracted are less than remaining events, defaults to 0,
    // else could trigger indefinate repeats. If indefiniate repeat is desired, use startDelay(-1) or startIndefinate();
    // Adding additional events to an indefinate delay event could lead to incorrect values. Best to use startDelay(X);




    // Depreciated Methods

    bool delayScript(uint32_t delayTime);            // Old delay function left in to maintain backwards compatability


    // Variables

    bool delayActive = false;

    int16_t eventCounter = 0;

  private:

    //Methods

    bool globalDelay(uint32_t currentTime, uint32_t delayTime);          // Global delay method called by all other methods (as architecture is largely the same)

    bool delayEvent(uint32_t currentTime, uint32_t delayTime);           // Global delayEvent method called by all other delayEvent methods

    // Variables
    long previousTime;


};

#endif
