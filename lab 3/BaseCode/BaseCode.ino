/******************************************
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 44 - left stepper motor step pin
  digital pin 49 - left stepper motor direction pin
  digital pin 46 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin

  analog pin 0 - Back IR Sensor
  analog pin 1 - Left IR Sensor
  analog pin 2 - Front IR Sensor
  analog pin 3 - Right IR Sensor
  
  analog pin 5 - Left Sonar Sensor
  analog pin 6 - Right Sonar Sensor
  
*/

/*******************************LIBRARIES*******************************************/
#include <AccelStepper.h>   //include the stepper motor library
#include <NewPing.h>        //include sonar ping library

/*******************************#DEFINES*******************************************/
// Stepper
#define rtStepPin 44        //right stepper motor step pin
#define rtDirPin 49         //right stepper motor direction pin
#define ltStepPin 46        //left stepper motor step pin
#define ltDirPin 53         //left stepper motor direction pin
#define stepperEnable 48    //stepper enable pin on stepStick 
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

// IR Definitions
#define irF 0       //back IR
#define irR 1       //left IR
#define irB 2       //front IR
#define irL 3       //right IR

// Sonar Definitions
#define sPinL A8            //left sonar pin
#define sPinR A9            //right sonar pin

// Stepper Library Default Speeds
#define speedD 300          //default speed
#define accelD 1200         //default acceleration
#define updateDist 12800    //

// LED Pin Definitions
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states

// Code Operation
#define pauseTime 2500      //time before robot moves

// Sensor reading values
#define cutoff 15           //cutoff for reading IR sensor values



/*******************************VARIABLES*******************************************/
// Variables for converting steps to distances

const int spr = 800;                  //stepper steps per revolution of the wheel
const double d = 3.365;               //diameter of the wheel in inches
const double w = 8.25;                //distance between centers of wheels in inches
const double dstStep = PI * d / spr;  //linear distance of one wheel step


/*******************************INIT***********************************************/
// Sonar Setup
NewPing sonarL(sPinL, sPinL, 200);            //setup left sonar
NewPing sonarR(sPinR, sPinR, 200);            //setup right sonar

// Stepper Setup
AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)



void setup() {
  // Added to improve debugging capability
  Serial.begin(9600); // open the serial port at 9600 baud

  // Pin initialization
  pinMode(rtStepPin, OUTPUT);                     //sets pin as output
  pinMode(rtDirPin, OUTPUT);                      //sets pin as output
  pinMode(ltStepPin, OUTPUT);                     //sets pin as output
  pinMode(ltDirPin, OUTPUT);                      //sets pin as output
  pinMode(stepperEnable, OUTPUT);                 //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);    //turns off the stepper motor driver
  pinMode(redLED, OUTPUT);                        //set red LED as output
  pinMode(grnLED, OUTPUT);                        //set green LED as output
  pinMode(ylwLED, OUTPUT);                        //set yellow LED as output

  // Stepper initialization
  stepperRight.setMaxSpeed(speedD);               //set the maximum speed for the right stepper
  stepperLeft.setMaxSpeed(speedD);                //set the maximum speed for the left stepper
  stepperRight.setAcceleration(accelD);           //set the initial acceleration for the right stepper
  stepperLeft.setAcceleration(accelD);            //set the initial acceleration for the right stepper
  steppers.addStepper(stepperRight);              //add right motor to MultiStepper steppers
  steppers.addStepper(stepperLeft);               //add left motor to MultiStepper steppers
  digitalWrite(stepperEnable, stepperEnTrue);     //turns on the stepper motor driver
  delay(pauseTime);                               //delay before the robot moves
  
}

void loop() {
  drive();

}

/*
 * drive() function recurcively calls itself and makes sure that our drive motors are always moving
 */
void drive(){
  if (!stepperRight.runSpeed()) {
    stepperRight.move(updateDist);
  }
  if (!stepperLeft.runSpeed()) {
    stepperLeft.move(updateDist);
  }
  drive();
}



/*
 * readIRRight returns the value of the right IR sensor in inches
*/
double readIRRight() {
  double A = analogRead(irR);               //read the sensor value
  double val = (2421.5 / (A + 1.0)) - 1.92; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}


/*
 * readIRLeftrns the value of the left IR sensor in inches
*/
double readIRLeft) {
  double A = analogRead(irL);               //read the sensor value
  double val = (2495.0 / (A + 3.0)) - 1.80; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}

/*
 * readIRFront returns the value of the front IR sensor in inches
*/
double readIRFront() {
  double A = analogRead(irF);               //read the sensor value
  double val = (1049.0 / (A + 7.0)) - 0.23; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}

/*
 * readIRBack returns the value of the back IR sensor in inches
*/
double readIRBront() {
  double A = analogRead(irB);               //read the sensor value
  double val = (1072.0 / (A + 1.0)) - 0.38; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}

/*
 * readSonL returns the value of the left sonar sensor in inches
*/
double readSonR() {
  unsigned int val = sonarR.ping();
  val = 0.0071 * val - 0.2686;
  if (val > cutoff) {
    val = cutoff;
  }
  return (val);
}

/*
 * readSonR returns the value of the right sonar sensor in inches
*/
double readSonL() {
  unsigned int val = sonarL.ping();
  val = 0.0069 * val - 0.0544;
  if (val > cutoff) {
    val = cutoff;
  }
  return (val);
}


/*
 * setLED recieves a string including the characters of LEDs that should be turned on.
 * Characters include
 * 'R' --- Red LED
 * 'Y' --- Yellow LED
 * 'G' --- Green LED
*/
void setLED(String led) {
  resetLED();                   //reset the LEDs
  if (led.indexOf('R') >= 0) {  //if the string input contains the character 'R'
    digitalWrite(redLED, HIGH); //set the red LED to HIGH
  }
  if (led.indexOf('Y') >= 0) {  //if the string input contains the character 'Y'
    digitalWrite(ylwLED, HIGH); //set the yellow LED to HIGH
  }
  if (led.indexOf('G') >= 0) {  //if the string input contains the character 'G'
    digitalWrite(grnLED, HIGH); //set the green LED to HIGH
  }
}

/*
 * resetLED turns all three leds to the LOW state.
*/
void resetLED(void) {
  digitalWrite(redLED, LOW);  //reset the red LED
  digitalWrite(ylwLED, LOW);  //reset the yellow LED
  digitalWrite(grnLED, LOW);  //reset the green LED
}

/*
 * convert a value from inches/XX to steps/XX 
 * val is a value with units of inches/XX
*/
int convertStp (double val) {
  int ans = (int) ceil(val / dstStep); //conversion factor
  return ans;   //return ans
}
