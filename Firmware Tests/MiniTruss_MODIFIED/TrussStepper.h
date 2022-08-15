/*
 * TrussStepper library
 *
	Author: David Reid
 *
 Based upon the Arduino Stepper library:
 https://www.arduino.cc/en/Reference/Stepper
 
 For remote control of the NEMA 17 bipolar stepper motor with microstepping driver.
 Library only works for 2 pin motor control - direction + pulse pins.
 */

// ensure this library description is only included once
#ifndef TrussStepper_h
#define TrussStepper_h

#include <Arduino.h>

// library interface description
class TrussStepper {
  public:
    // constructor:
    TrussStepper(int steps_per_rev, int direction_pin, int pulse_pin, int enable_pin);
    TrussStepper(int steps_per_rev, int direction_pin, int pulse_pin, int enable_pin, double duty_cycle);
    
    // for setting the enable pin - depowering the stepper when not moving
    void enable();
    void disable();
    
    // speed setter method:
    void setDelay(long delay);

    // move multiple steps:
    void step(int steps_to_move);

  private:
    void singleStep();

    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in ms, based on speed
    int steps_per_rev;      // total number of steps in one revolution of this motor
    int step_number;          // which step the motor is on
    float duty_cycle;
    // motor pin numbers:
    int direction_pin;
    int pulse_pin;
    int enable_pin;

    unsigned long last_step_time; // time stamp in micro-seconds of when the last step was taken
};

#endif
