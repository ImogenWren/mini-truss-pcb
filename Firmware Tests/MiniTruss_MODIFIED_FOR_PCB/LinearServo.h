/*
 * LinearServo library
 *
	Author: David Reid
 *
 
 For remote control of the Actuonix L16-50-150-6-R linear actuator.
 Library only works for 3 pin servo - signal pin, +VCC, GND
 */

// ensure this library description is only included once
#ifndef LinearServo_h
#define LinearServo_h

#include <Arduino.h>

// library interface description
class LinearServo {
  public:
    // constructor:
    LinearServo(int signal_pin, int max_position);
    LinearServo(int signal_pin, int max_position, long delay);
    
    // speed setter method:
    void setDelay(long delay);

    void updateMoveTo(int moveTo);
    int update();
    
    void zero();

  private:
    void pulse(int position);
    unsigned long delay; // delay between steps, in micros
    int current_position;
    int move_position;		
    int max_pos;
    
    // motor pin numbers:
    int signal_pin;
    
};

#endif
