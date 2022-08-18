/*
 * LinearServo library
 *
	Author: David Reid
 *
 
 For remote control of the Actuonix L16-50-150-6-R linear actuator.
 Library only works for 3 pin servo - signal pin, +VCC, GND
 
Control linear servo position with a single digital pin.

Position is set between 0 (full retraction) and 100 (full extension).
 */

#include <Arduino.h>
#include "LinearServo.h"

/*
 * one-wire constructor.
 * Sets which pin should control the motor.
 */
LinearServo::LinearServo(int signal_pin, int max_position)
{
  this->delay = 10000L;		//step period in microseconds. Default value
  
  // Arduino pins for the motor control connection:
  this->signal_pin = signal_pin;

  // setup the pins on the microcontroller:
  pinMode(this->signal_pin, OUTPUT);
  
  this->current_position = 0;		//between 0(full retraction) and 100(full extension)
  this->move_position = 0;
  this->max_pos = max_position;	//the limit of servo extension

}

/*
 * one-wire constructor.
 * Sets which pin should control the motor.
 * Can also set the pulse frequency by setting a custom delay.
 */
LinearServo::LinearServo(int signal_pin, int max_position, long delay)
{
	if(delay > 2000L)
	{
		this->delay = delay;		
  	} 
  	else
  	{
  		this->delay = 2000L;
  	}
  	
  // Arduino pins for the motor control connection:
  this->signal_pin = signal_pin;

  // setup the pins on the microcontroller:
  pinMode(this->signal_pin, OUTPUT);
  
  this->current_position = 0;		//between 0(full retraction) and 100(full extension)
  this->move_position = 0;
  this->max_pos = max_position;	//the limit of servo extension
}

/*
 * Sets the delay between steps in millseconds -> effectively the speed of the stepper
 */
void LinearServo::setDelay(long whatDelay)
{
	if(whatDelay > 2000L)
	{
		this->delay = whatDelay;		//step period in microseconds. Default value
  	} 
  	else
  	{
  		this->delay = 2000L;
  	}
}

/*
 * Zeros the servo, returning it to fully retracted
 */
void LinearServo::zero()
{
	updateMoveTo(0);
	this->current_position = this->max_pos;
	
	for(int i=this->max_pos;i>=0;i--)
	{
		update();
		delayMicroseconds(100000);
  	}
  	
  	this->current_position = 0;
  	
}
/*
* Update the target position that we want the servo to move to.
*/

void LinearServo::updateMoveTo(int moveTo)
{
	if(moveTo < 0)
	{
		this->move_position = 0;
	} 
	else if(moveTo <= this->max_pos)
	{
		this->move_position = moveTo;
	} 
	else 
	{
		this->move_position = this->max_pos;
	}
	
	
	

}
/*
 * Sets the position of the servo between 0 (fully retracted) and 100 (fully extended).
 * A pulse of 1ms causes full retraction; 2ms causes full extension.
 * 
 */
int LinearServo::update()
{
	
	if(this->move_position > this->current_position)
	{
	    this->current_position += 1;
	    pulse(this->current_position);
	    
	  } 
	  else if(this->move_position < this->current_position)
	  {
	    this->current_position -= 1;
	    pulse(this->current_position);
	   
	  } 
	  else 
	  {
	  	pulse(this->current_position);
	  }
	  
	  return this->current_position;
}

/*
 * Sends a pulse of appropriate length for position.
 */
void LinearServo::pulse(int position)
{
    	unsigned long high_delay = 1000+(position*10);	//pulse length between 1ms and 2ms
	unsigned long low_delay = this->delay - high_delay;	//the remainder of delay
	digitalWrite(this->signal_pin, HIGH);
	delayMicroseconds(high_delay);
	digitalWrite(this->signal_pin, LOW);
	delayMicroseconds(low_delay);
    
  
}
