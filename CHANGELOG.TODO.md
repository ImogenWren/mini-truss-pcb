#Changelog & TODO


#TODO PCB changes for V2:

1. Add GND Test points / additional pins 
2. Pull Down Resistors on logic level converter spare pins
3. Add spare voltage pins for devwork and testing
4. GND to connecter housing?
5. GND to mounting holes? Nope dont like - introduces ground loop
6. Add labels to headers
7. Change reference to 680uF cap
8. route LEDs to connector
9. change minimelf to reflect negative
10. add dot to IC footprint


#TODO Additional connector board req
- Breakout board for quick connection of individual sensors to 37 pin connector

# Servo Notes

Option R – RC Linear Servo
WIRING: (see last page for pin numbering)
1 - White – RC input signal (RC-servo compatible)
2 - Red – Power (+6 VDC)
3 - Black – Ground

Note: Reversing the polarity of pins 2 and 3 may permanently
damage the actuator
The –R actuators or ‘linear servos’ are a direct replacement for
regular radio controlled hobby servos. The desired actuator
position is input to the actuator on lead 1 as a positive 5 Volt
pulse width signal. A 1.0 ms pulse commands the controller
to fully retract the actuator, and a 2.0 ms pulse signals it to
fully extend. If the motion of the actuator, or of other servos
in your system, seems erratic, place a 1–4Ω resistor in series
with the actuator’s red V+ lead wire.
The –R actuators are available in 6 volt and 30, 50 and 100 mm
strokes only.
This is a lower cost option for the RC hobby market. While the
supported control options are reduced, the –R retains the same
great precision and cycle life as the –I. 

