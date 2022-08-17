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



//JSON serialization
#define COMMAND_SIZE 64  //originally 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

//STEPPER VARIABLES
#define SEN 14
#define SDIR 15

//#define SPUL 16
#define SPUL 12      // Changed definition of the pulse pin


const int stepperStepsPerRev = 200;
const int stepperStepPeriod = 1000; //microseconds
TrussStepper stepper = TrussStepper(stepperStepsPerRev, SDIR, SPUL, SEN);
int currentPos = 0;     //the position of the stepper in terms of number of steps
int moveToPos = 0;      //the position the stepper should move to in terms of steps.
const int positionLimit = 100*stepperStepsPerRev;
const int direction = -1;   //reverse the direction of the steps -> -1
bool isStepperEnabled = false;

//LIMIT SWITCHES
bool limitSwitchesAttached = true;
#define limitSwitchLower 11
bool lowerLimitReached = false;
#define limitSwitchUpper 13
bool upperLimitReached = false;

//GAUGE READINGS
const int numGauges = 7;
float data[numGauges] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//GAUGE SETUP
const int SCK_PIN = 2;  //Common CLOCK pin
const int GAUGE_0_DT = 3; //DATA pins
const int GAUGE_1_DT = 4;
const int GAUGE_2_DT = 5;
const int GAUGE_3_DT = 6;
const int GAUGE_4_DT = 7;
const int GAUGE_5_DT = 8;
const int GAUGE_6_DT = 9;

const int pins[numGauges] = {GAUGE_0_DT, GAUGE_1_DT, GAUGE_2_DT, GAUGE_3_DT, GAUGE_4_DT, GAUGE_5_DT, GAUGE_6_DT};

HX711 gauge_0;
HX711 gauge_1;
HX711 gauge_2;
HX711 gauge_3;
HX711 gauge_4;
HX711 gauge_5;
HX711 gauge_6;

//these will need to be calibrated for new truss setup
const int scale_load = -15184;
const int scale_factor_1 = 4050;
const int scale_factor_2 = scale_factor_1*1.030;
const int scale_factor_3 = scale_factor_1*0.977;
const int scale_factor_4 = scale_factor_1*0.947;
const int scale_factor_5 = scale_factor_1*0.818;
const int scale_factor_6 = scale_factor_1*0.924;

HX711 gauges[numGauges] = {gauge_0, gauge_1, gauge_2, gauge_3, gauge_4, gauge_5, gauge_6};

//TIMING FOR GAUGE WRITING
unsigned long timeInterval = 1000;    //write out gauge readings with a period no smaller than 1s
unsigned long previousTime = 0;

/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_STANDBY = 0,        //no drive to motor, no reading of gauges
  STATE_READ = 1,           //reads each gauge
  STATE_WRITE = 2,            //writes to serial
  STATE_MOVE = 3,           //allows stepper motor to move to new position
  STATE_ZERO = 4,           //zeroes the position of the servo
  STATE_TARE_GAUGES = 5,           //tares (zeroes) the gauge readings
  STATE_TARE_LOAD = 6,      //tares the load force gauge
  STATE_TARE_ALL = 7,      //tares both the gauges and load cell
  STATE_GAUGE_RESET = 8,    //resets all gauges
  
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Standby(void);
void Sm_State_Read(void);
void Sm_State_Write(void);
void Sm_State_Move(void);
void Sm_State_Zero(void);
void Sm_State_Tare_Gauges(void);
void Sm_State_Tare_Load(void);
void Sm_State_Tare_All(void);
void Sm_State_Gauge_Reset(void);

/**
 * Type definition used to define the state
 */
 
typedef struct
{
  StateType State; /**< Defines the command */
  void (*func)(void); /**< Defines the function to run */
} StateMachineType;

/**
 * A table that defines the valid states of the state machine and
 * the function that should be executed for each state
 */
StateMachineType StateMachine[] =
{
  {STATE_STANDBY, Sm_State_Standby},
  {STATE_READ, Sm_State_Read},
  {STATE_WRITE, Sm_State_Write},
  {STATE_MOVE, Sm_State_Move},
  {STATE_ZERO, Sm_State_Zero},
  {STATE_TARE_GAUGES, Sm_State_Tare_Gauges},
  {STATE_TARE_LOAD, Sm_State_Tare_Load},
  {STATE_TARE_ALL, Sm_State_Tare_All},
  {STATE_GAUGE_RESET, Sm_State_Gauge_Reset},
};
 
int NUM_STATES = 9;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_READ;    //START IN THE READ STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================

//TRANSITION: STATE_STANDBY -> STATE_STANDBY
void Sm_State_Standby(void){

  if(isStepperEnabled)
  {
    stepper.disable();
    isStepperEnabled = false;
  }

  //is there a need to detach these interrupts? Best to just attach and keep attached?
  if(limitSwitchesAttached)
  {
    detachInterrupt(digitalPinToInterrupt(limitSwitchLower));
    detachInterrupt(digitalPinToInterrupt(limitSwitchUpper));

    limitSwitchesAttached = false;
  }

  
  SmState = STATE_STANDBY;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ READ ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_READ -> STATE_READ
// State Read loops through all the gauges, reading their values and storing for writing.
// Remains in read state until user makes the change.
void Sm_State_Read(void){

  upperLimitReached = false;    //if in read state then clear the limit flags.    =====NEW
  lowerLimitReached = false;
  
  if(isStepperEnabled)
  {
    stepper.disable();
    isStepperEnabled = false;
  }

  if(millis() - previousTime >= timeInterval){
  
   for(int i=0; i< numGauges; i++){
      //if(gaugeScales[i].wait_ready_timeout(100)){
      if(gauges[i].is_ready()){
        
        data[i] = gauges[i].get_units(5);       //what is the best number of readings to take?
        
      } 

      delay(10);    //necessary?
   }

    report();
    previousTime = millis();
    
  }
  
   SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ WRITE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// NOT REACHED FROM ANY STATE NOR CAN USER SELECT IT ===== REMOVE
//TRANSITION: STATE_WRITE -> STATE_READ
// STATE_WRITE simply reports the stored gauge values by writing to serial.
void Sm_State_Write(void){
  
  upperLimitReached = false;    //if in read state then clear the limit flags.    =====NEW
  lowerLimitReached = false;
  
  if(isStepperEnabled)
  {
    stepper.disable();
    isStepperEnabled = false;
  }

  if(millis() - previousTime >= timeInterval){
    report();
    previousTime = millis();
  }

  SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MOVE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_MOVE -> STATE_READ
//Remains in move state until current position matches moveTo position.
//This blocks gauge reading, but high stepper speed and slow update of gauges should make this fine.
void Sm_State_Move(void){

  bool up = true;
  
  if(!isStepperEnabled)
  {
    stepper.enable();
    isStepperEnabled = true;
  }
  
  if(!limitSwitchesAttached)
  {
    attachInterrupt(digitalPinToInterrupt(limitSwitchLower), doLimitLower, FALLING);
    attachInterrupt(digitalPinToInterrupt(limitSwitchUpper), doLimitUpper, FALLING);

    limitSwitchesAttached = true;
  }

  if(lowerLimitReached || upperLimitReached)
  {
    lowerLimitReached = false;
    upperLimitReached = false;
  }
  
  if(moveToPos != currentPos)
  {
    if(currentPos > moveToPos)
    {
      //step clockwise with stepper class
      stepper.step(-1*direction);    //might want to put a direction offset in the library
      currentPos -= 1;
      up = false;
    } 
    else if(currentPos < moveToPos)
    {
      //step anticlockwise with stepper class
      stepper.step(1*direction);
      currentPos += 1;
      up = true;  
    }

    //stay in move state until moveToPos == currentPos
    SmState = STATE_MOVE;
    
  }
  else
  {
    //current position has reached the requested moveTo position so can go back to reading the gauges.
    SmState = STATE_READ;
  }
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ZERO ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_ZERO -> STATE_READ
//Move to the upper limit switch and then makes a fixed number of steps downwards and sets this as 0 position.
void Sm_State_Zero(void){

  if(!limitSwitchesAttached)
  {
    attachInterrupt(digitalPinToInterrupt(limitSwitchLower), doLimitLower, FALLING);
    attachInterrupt(digitalPinToInterrupt(limitSwitchUpper), doLimitUpper, FALLING);

    limitSwitchesAttached = true;
  }
  
  if(!isStepperEnabled)
  {
    stepper.enable();
    isStepperEnabled = true;
  }
  
  if(!upperLimitReached)
  {
    stepper.step(-1*direction);
    currentPos -= 1;      //not necessary?
    SmState = STATE_ZERO;
  }
  else 
  {
    SmState = STATE_READ;
  }
 
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ TARE GAUGES++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_TARE_GAUGES -> STATE_READ
void Sm_State_Tare_Gauges(void){

  if(isStepperEnabled)
  {
    stepper.disable();
    isStepperEnabled = false;
  }
  
  tareGauges();
  delay(100);
  
  SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ TARE LOAD ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_TARE_LOAD -> STATE_READ
void Sm_State_Tare_Load(void){

  if(isStepperEnabled)
  {
    stepper.disable();
    isStepperEnabled = false;
  }
  
  tareLoad();
  delay(100);
  
  SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ TARE ALL ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_TARE_ALL -> STATE_READ
void Sm_State_Tare_All(void){

  if(isStepperEnabled)
  {
    stepper.disable();
    isStepperEnabled = false;
  }
  
  tareAll();
  delay(100);
  
  SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ GAUGE RESET ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_GAUGE_RESET -> STATE_READ
void Sm_State_Gauge_Reset(void){

  if(isStepperEnabled)
  {
    stepper.disable();
    isStepperEnabled = false;
  }
  
  resetGauges();
  delay(100);
  
  SmState = STATE_READ;
  
}

//STATE MACHINE RUN FUNCTION
void Sm_Run(void)
{
  if (SmState < NUM_STATES)
  {
    SmState = readSerialJSON(SmState);      
    (*StateMachine[SmState].func)();        //reads the current state and then runs the associated function
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}



void setup() {

  pinMode(limitSwitchLower, INPUT_PULLUP);
  pinMode(limitSwitchUpper, INPUT_PULLUP);
  pinMode(OUTPUT_ENABLE, OUTPUT);

  digitalWrite(OUTPUT_ENABLE, HIGH);
  led_setup();
  led_poweron();

  previousTime = millis();

  //Serial communication for sending data -> RPi -> Server
  Serial.begin(57600);
  while(!Serial);

  stepper.setDelay(stepperStepPeriod);
  stepper.disable();
  isStepperEnabled = false;

  resetGauges();

}

void loop() {

  Sm_Run();

}


StateType readSerialJSON(StateType SmState){
  if(Serial.available() > 0)
  {

    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);
    
    const char* set = doc["set"];

    if(strcmp(set, "position")==0)
    {
  
        float new_position = doc["to"];
        
        if(new_position >= -positionLimit && new_position <= positionLimit)
        {
          moveToPos = new_position;
        } 
        else
        {
          Serial.println("Outside position range");
        }
     
  } 
    else if(strcmp(set, "mode")==0)
    {
      
      const char* new_mode = doc["to"];

        if(strcmp(new_mode, "standby") == 0)
        {
          SmState = STATE_STANDBY;
          reportState(STATE_STANDBY);//necessary?
        } 
        else if(strcmp(new_mode, "move") == 0)
        {
          SmState = STATE_MOVE;
          reportState(STATE_MOVE);//necessary?
        }
        else if(strcmp(new_mode, "zero") == 0)
        {
          SmState = STATE_ZERO;
          reportState(STATE_ZERO);//necessary?
        }
        else if(strcmp(new_mode, "tare") == 0)
        {
          SmState = STATE_TARE_GAUGES;
          reportState(STATE_TARE_GAUGES);//necessary?
        }
        else if(strcmp(new_mode, "tare_load") == 0)
        {
          SmState = STATE_TARE_LOAD;
          reportState(STATE_TARE_LOAD);   //necessary?
        }
        else if(strcmp(new_mode, "tare_all") == 0)
        {
          SmState = STATE_TARE_ALL;
          reportState(STATE_TARE_ALL);   //necessary?
        }
        else if(strcmp(new_mode, "gauge_reset") == 0)
        {
          SmState = STATE_GAUGE_RESET;
          reportState(STATE_GAUGE_RESET);//necessary?
        }
        
    }  
    
  }
      return SmState;     //return whatever state it changed to or maintain the state.
 } 

 //On an interrupt - will interrupt all state functions
//TRANSITION: -> READ
void doLimitLower(void){
  if(!lowerLimitReached)
  {
    //if the lower limit is reached, then the stepper should move a small distance back towards the centre, away from the limit
    lowerLimitReached = true;
    
    //TEMP OUTPUT OF DATA
    //Serial.print("Lower limit at pos: ");
    //Serial.println(currentPos);
      
    stepper.step(-15*stepperStepsPerRev*direction);
    moveToPos = currentPos;
    SmState = STATE_READ; 
  }
}

//On an interrupt - will interrupt all state functions
//TRANSITION: -> READ
void doLimitUpper(void){
  if(!upperLimitReached)
  {
    upperLimitReached = true;

    //TEMP OUTPUT OF DATA
    //Serial.print("Upper limit at pos: ");
    //Serial.println(currentPos);

    stepper.step(15*stepperStepsPerRev*direction);
    currentPos = 0;
    moveToPos = 0;
    SmState = STATE_READ;  
  }
}


/*
void newReport(){
char temp_char[][];

for (int i = 0; i < numGauges+1; i++){
   dtostrf(data[i], 4, 4, c_buff[i]);  //4 is mininum width, 6 is precision
}

  
char out_buffer[128];
sprintf(out_buffer, "{\"load_cell\": %s, \"gauge_1\" :");
  
}
*/

int iteration =0;

void report(){
  
  Serial.print("{\"load_cell\":");
  Serial.print(data[0]);
  
  for(int i=1;i<numGauges;i++){
    Serial.print(",\"gauge_");
    Serial.print(i);
    Serial.print("\":");
    Serial.print(data[i]);
  }

  Serial.print(",\"state\":");
  Serial.print(SmState);
  Serial.print(",\"pos\":");
  Serial.print(currentPos);
  
  Serial.print("}");

  Serial.print(" Iteration: ");
  Serial.print(iteration);
  iteration++;
  Serial.println();
  
 
}

//DO I NEED this function when state is being reported in report()?
void reportState(int state){
  Serial.print("{\"state\":");
  Serial.print(state);
  Serial.println("}");
}

void initialiseGauges(){
  for(int i=0;i<numGauges;i++){
    gauges[i].begin(pins[i], SCK_PIN);
  }
}

void setGain(int gain){
  for(int i=0;i<numGauges;i++){
    gauges[i].set_gain(gain);
  }
}

//Just tares the load cell
void tareLoad(){
  data[0] = 0.0;
  gauges[0].tare();
}

//tares all gauge scales, but not load cell
void tareGauges(){
  for(int i=1;i<numGauges;i++){
    data[i] = 0.0;       //set the stored data value to 0
    gauges[i].tare();
   }
}

//tares all gauges, including load cell
void tareAll(){
  for(int i=0;i<numGauges;i++){
    data[i] = 0.0;       //set the stored data value to 0
    gauges[i].tare();
   }
}

void resetGauges(){
  initialiseGauges();
  setGain(128);
 
  gauges[0].set_scale(scale_load);   //calibrated with the load cell on the real truss -> OUTPUTS force in newtons
  gauges[1].set_scale(scale_factor_1);          //member 1, calibrated with truss member 1  -> outputs strain in micro-strain
  gauges[2].set_scale(scale_factor_2);          //member 2
  gauges[3].set_scale(scale_factor_3);          //member 3
  gauges[4].set_scale(scale_factor_4);          //member 4
  gauges[5].set_scale(scale_factor_5);          //member 5
  gauges[6].set_scale(scale_factor_6);          //member 6

  
  tareAll();

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
