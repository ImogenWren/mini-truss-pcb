//Firmware for MiniTruss remote lab experiment
// Single controller firmware with state machine for reading and writing.

//Author: David P. Reid
//dprydereid@gmail.com
// 17/03/22

// IMPORT LIBRARIES
#include "HX711.h"
#include "TrussStepper.h"
#include "ArduinoJson-v6.9.1.h"


// Imogen's Additions
// Set Output Enable pin for logic level shifters

#define OUTPUT_ENABLE A1   // Must be set high if using Logic Level Shifters
//Additional LED Outputs
#define LED_ONE       13
#define LED_NINE      A5
#define LED_TEN       A6
#define LED_ELEVEN    A7





void setup() {
  led_setup();
  led_poweron();
}


void loop() {


}




#define LED_BLUE LED_ONE
#define LED_GREEN LED_ELEVEN
#define LED_YELLOW LED_TEN
#define LED_RED LED_NINE



char led_array[4] = {LED_BLUE, LED_GREEN, LED_YELLOW, LED_RED};

void led_setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(led_array[i], OUTPUT);
  }
}

void led_poweron() {
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(led_array[i], HIGH);
      delay(70);
    }
    for (int i = 0; i < 4; i++) {
      digitalWrite(led_array[i], LOW);
      delay(70);
    }
  }
}
