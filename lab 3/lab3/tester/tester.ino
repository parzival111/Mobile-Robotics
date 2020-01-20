#include <AccelStepper.h>   //include the stepper motor library
#include <MultiStepper.h>   //include multiple stepper motor library

const int rtStepPin = 44;             //right stepper motor step pin
const int rtDirPin = 49;              //right stepper motor direction pin
const int ltStepPin = 46;             //left stepper motor step pin
const int ltDirPin = 53;              //left stepper motor direction pin

const int straight = 3;               //default movement distance for going forward
const int dSpd = 4;                   //default speed for movement


const int spr = 800;                  //stepper steps per revolution of the wheel
const double d = 3.365;               //diameter of the wheel in inches
const double w = 8.25;                //distance between centers of wheels in inches
const double dstStep = PI * d / spr;  //linear distance of one wheel step

// position tracking
double xPos = 0;                      //x position of the robot
double yPos = 0;                      //y position of the robot
double angle = 0;                     //heading of the robot
double xGoal = 0;                     //x coordinate of the goal
double yGoal = 0;                     //y coordinate of the goal

AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                  //create instance to control multiple steppers at the same time


#define stepperEnable 48    //stepper enable pin on stepStick 
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define speedD 300          //default speed
#define accelD 1200          //default acceleration

#define pauseTime 2500 //time before robot moves


void setup() {
  // put your setup code here, to run once:


  // Stepper initialization
  stepperRight.setMaxSpeed(speedD);               //set the maximum speed for the right stepper
  stepperLeft.setMaxSpeed(speedD);                //set the maximum speed for the left stepper
  stepperRight.setAcceleration(accelD);           //set the initial acceleration for the right stepper
  stepperLeft.setAcceleration(accelD);            //set the initial acceleration for the right stepper
  steppers.addStepper(stepperRight);              //add right motor to MultiStepper steppers
  steppers.addStepper(stepperLeft);               //add left motor to MultiStepper steppers
  digitalWrite(stepperEnable, stepperEnTrue);     //turns on the stepper motor driver

  delay(pauseTime);
}

void loop() {
  // put your main code here, to run repeatedly:

  drive(straight, dSpd);

}


/*
    drive() moves the robot straight at the speed recieved from its input.
*/
void drive(int dist, int spd) {
  spd = convertStp(spd);          //convert speeds from input to steps/sec]
  xPos += dist * cos(angle * PI / 180.0);
  yPos += dist * sin(angle * PI / 180.0);
  dist = convertStp(dist);        //convert distances from inches to steps
  stepperLeft.move(-dist);         //set stepper distance
  stepperRight.move(-dist);        //set stepper distance
  stepperLeft.setMaxSpeed(spd);   //set stepper speed
  stepperRight.setMaxSpeed(spd);  //set stepper speed
  runToStop();                    //move to the desired position
}


/*
   runToStop runs both the right and left stepper until they stop moving
*/
void runToStop ( void ) {
  int runL = 1;   //state variabels
  int runR = 1;   //state variables
  while (runL || runR) {        //until both stop
    if (!stepperRight.run()) {  //step the right stepper, if it is done moving set runR = 0
      runR = 0;                 //left done moving
    }
    if (!stepperLeft.run()) {   //step the left stepper, if it is done moving set runL = 0
      runL = 0;                 //right done moving
    }
  }
}

/*
  convert a value from inches/XX to steps/XX or the right wheel

  val is a value with units of inches/XX
*/
int convertStp (double val) {
  int ans = (int) ceil(val / dstStep); //conversion factor
  return ans;   //return ans
}
