#include <Arduino.h>
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DO0_PIN = 3;
const int LOADCELL_DO1_PIN = 4;
const int LOADCELL_DO2_PIN = 5;
const int LOADCELL_DO3_PIN = 6;
const int LOADCELL_DO4_PIN = 7;
const int LOADCELL_DO5_PIN = 8;
const int LOADCELL_DO6_PIN = 9;
const int LOADCELL_SCK_PIN = 2;

int sensorPinArray[7] = {3, 4, 5, 6, 7, 8, 9};

HX711 sensorArray[7];



void setup() {
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);
  Serial.begin(57600);
  while (!Serial){
    delay(10);
  }
  Serial.println("HX711 Demo");
  Serial.println("Initializing the scale");

  for(int i = 0; i < 8; i++){
    sensorArray[i].begin(sensorPinArray[i], LOADCELL_SCK_PIN);
  }
 
for(int i = 0; i < 8; i++){
  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(sensorArray[i].read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(sensorArray[i].read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(sensorArray[i].get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(sensorArray[i].get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)
            
  sensorArray[i].set_scale(-459.542);
  //scale.set_scale(-471.497);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  sensorArray[i].tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");


  Serial.print("read: \t\t");
  Serial.println(sensorArray[i].read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(sensorArray[i].read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(sensorArray[i].get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(sensorArray[i].get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");
}
}

void Calibrate() {
for(int i = 0; i < 8; i++){
  Serial.print("Calibrating Sensor Number: ");
  Serial.println(i);
  if (sensorArray[i].is_ready()) {
    sensorArray[i].set_scale();    
    Serial.println("Tare... remove any weights from the sensorArray[i].");
    delay(5000);
    sensorArray[i].tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(5000);
    long reading = sensorArray[i].get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
  } 
  else {
    Serial.println("HX711 not found.");
  }
}
  delay(1000);
}

void loop() {
   Serial.print("Raw Read:\t");
for(int i = 0; i < 8; i++){
   Serial.println(sensorArray[i].read());                 // print a raw reading from the ADC
 // Serial.print(sensorArray[i].get_units(), 1);
 // Serial.print("\t| average:\t");
 // Serial.print(sensorArray[i].get_units(10), 5);
  Serial.print(", ");
}

Serial.println();
  delay(5000);
}


//void printValues(valArray[8]){
//buffer[64];
//sprintf(buffer, "Beams: %i,%i ", valArray[0], valArray[1], valArray[2], valArray[3], valArray[4], valArray[5], valArray[6]);
//Serial.println(buffer); 

//}
