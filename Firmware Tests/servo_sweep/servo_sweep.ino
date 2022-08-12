/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 8 Nov 2013
  by Scott Fitzgerald
  https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>
#include <autoDelay.h>

autoDelay servoDelay;

#define SERVO_DELAYTIME_mS 500

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


#define LIMIT_SWITCH_PIN 11
#define SERVO_PIN 12

#define SERVO_LOWER_LIMIT 40   // As a Percentage
#define SERVO_UPPER_LIMIT 50   // 40 = 0 on this system - 40 - 45 seems safe but visible operation of linear actuator


int pos = SERVO_LOWER_LIMIT;    // variable to store the servo position




int percent_to_angle(int percent) {
  float decimal = percent / 100.0;
  float angle = 180 * decimal;
  //  Serial.print("Angle: ");
  //  Serial.print(angle);
  //  Serial.print("  As Int: ");
  int output = int(angle);
  Serial.println(output);
  return output;
}


void setup() {
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);
}

int step_amount = 1;


void run_servo() {
  int angle;
  myservo.write(pos);
  if (digitalRead(LIMIT_SWITCH_PIN)) {
    if (servoDelay.millisDelay(SERVO_DELAYTIME_mS)) {
      angle = percent_to_angle(pos);
      myservo.write(pos);              // tel;l servo to go to position in variable 'pos'
      pos = pos + step_amount;
      if (pos >= SERVO_UPPER_LIMIT) {
        step_amount = -1;
      } else if (pos <= SERVO_LOWER_LIMIT) {
        step_amount = 1;
      }
    }
  } else {
    Serial.println("Limit Switch Pressed");
  }
}




void loop() {
  run_servo();
  //percent_to_angle(50);
}
