/*--------------------------------------autoDelay Library Example--------------------------------


   Simple method for timing events without using interrupt based delay functions

    autoDelay != delay();

    functions include delays passed values in:
    Micro & Milliseconds,
    Seconds, and Minuites.
    
    Makes timing events a piece of cake.

    Written by Declan Heard. 09/08/2020
    
    Released into Public Domain
    Free to Distibute and Use.



    This Sketch used seperate instances of autoDelay to control the timing
    of two seperate functions independently.

    More can be added as nessissary, just give them each a unique name.


     This example uses depreciated function names, but library is now backwards compatable with older versions.
*/



//-----------------------------Timer Function, Must include----------------------------

#include <autoDelay.h>

autoDelay autodelay;    // set up autodelay instance. NO PARENTHESIS if no value to pass.

autoDelay autodelay2;    // set up 2nd autodelay instance if required



int timerOne = 500;    // Timer value - set to the length of delay required. 


int timerTwo = 750;    // Timer value - set to the length of delay required.


//-----------------------^^^---Timer Function, Must include----------------------------




//---------Other Variables--- Only used for Example Sketch-----------------------------

int RledPin = 4;                                 // Set the Output Pin - Red LED
int GledPin = 5;                                 // Set the Output Pin - Green LED

int RledState = LOW;                            // state of the LED pin - Red LED
int GledState = LOW;                            // state of the LED pin - Green LED




//---------------------------SETUP---------------------------------------

void setup() {

  pinMode(RledPin, OUTPUT);                       // sets up LED pins as output pin
  pinMode(GledPin, OUTPUT);                       // sets up LED pins as output pin

  autodelay.begin(9600);                          // function to set up serial communication. uncomment if unneeded

}
//------------------------^^^^ END OF SETUP------------------------------






//-----------------------------MAIN LOOP------------------------------------

void loop() {




  //-------------------IF Time ONE has elapsed -----------------------------------
  if (autodelay.delayScript(timerOne)) { // if this is true, at once  // do the thing //



   REDstateChange();

  }  


 //-------------------IF Time TWO has elapsed -----------------------------------
  if (autodelay2.delayScript(timerTwo) == true) { // if this is true, at once  // do the thing //
 

   GREENstateChange();



  }  // end of IF statement




//----------------These lines are executed during every loop------------------------------
  digitalWrite(RledPin, RledState);  // Write the LED state to the led pin.
  digitalWrite(GledPin, GledState);  // Write the LED state to the led pin.

}  // end of void loop

//-----------------------------------------------^^^ END OF MAIN LOOP-----------------------------------






void REDstateChange(){   // function to change the state of the LEDs when called

 if (RledState == HIGH) {

      RledState = LOW;

    } else if (RledState == LOW) {

      RledState = HIGH;
    }
  
}

void GREENstateChange(){   // function to change the state of the LEDs when called

 if (GledState == HIGH) {

      GledState = LOW;

    } else if (GledState == LOW) {

      GledState = HIGH;
    }
  
}
