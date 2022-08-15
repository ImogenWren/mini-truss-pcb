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

#include <Arduino.h>
#include "TrussStepper.h"

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
TrussStepper::TrussStepper(int steps_per_rev, int direction_pin, int pulse_pin, int enable_pin)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->steps_per_rev = steps_per_rev; // total number of steps per revolution for this motor
  this->duty_cycle = 0.1;	//10% duty cycle for stepping motor
  this->step_delay = 1000L;		//step period in microseconds. Default value
  
  // Arduino pins for the motor control connection:
  this->direction_pin = direction_pin;
  this->pulse_pin = pulse_pin;
  this->enable_pin = enable_pin;

  // setup the pins on the microcontroller:
  pinMode(this->direction_pin, OUTPUT);
  pinMode(this->pulse_pin, OUTPUT);
  pinMode(this->enable_pin, OUTPUT);

}

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
TrussStepper::TrussStepper(int steps_per_rev, int direction_pin, int pulse_pin, int enable_pin, double duty_cycle)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->steps_per_rev = steps_per_rev; // total number of steps per revolution for this motor
  this->duty_cycle = duty_cycle;	//duty cycle for stepping motor
  this->step_delay = 1000L;		//step period in microseconds. Default value
  
  // Arduino pins for the motor control connection:
  this->direction_pin = direction_pin;
  this->pulse_pin = pulse_pin;
  this->enable_pin = enable_pin;

  // setup the pins on the microcontroller:
  pinMode(this->direction_pin, OUTPUT);
  pinMode(this->pulse_pin, OUTPUT);
  pinMode(this->enable_pin, OUTPUT);

}

/*
* Sets the enable pin LOW in order to enable the stepper
*/
void TrussStepper::enable()
{
	digitalWrite(this->enable_pin, LOW);
	
}

/*
* Sets the enable pin HIGH in order to disable the stepper
*/
void TrussStepper::disable()
{
	digitalWrite(this->enable_pin, HIGH);
}

/*
 * Sets the delay between steps in microseconds -> effectively the speed of the stepper
 */
void TrussStepper::setDelay(long whatDelay)
{
  this->step_delay = whatDelay;
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void TrussStepper::step(int steps_to_move)
{
  int steps_left = abs(steps_to_move);  // how many steps to take

  // determine direction based on whether steps_to_move is + or -:
  if (steps_to_move > 0) 
  { 
  	this->direction = 1; 
  	digitalWrite(direction_pin, HIGH);
  } 
  else 
  {
  	this->direction = 0;
  	digitalWrite(direction_pin, LOW);
  }

  //run a single step for all steps_left
  for (int i=0;i<steps_left;i++)
  {
    singleStep();
  }
}

/*
 * Moves the motor a single step in the direction already defined.
 */
void TrussStepper::singleStep()
{
    
    digitalWrite(pulse_pin, HIGH);
    delayMicroseconds(this->duty_cycle*this->step_delay);
    digitalWrite(pulse_pin, LOW);
    delayMicroseconds((1.0-this->duty_cycle)*this->step_delay);
  
}
