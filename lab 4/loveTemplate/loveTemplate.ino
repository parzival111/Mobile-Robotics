/************************************
   Arbib-Lab03.ino
   Peter Garnache and Carson Stone 12/12/19

   This program demonstrates a state machine that implements wall following behaviors to navigate around a room

   The created functions include the following:
   drive() - continually call the stepper.runSpeed() functions to keep the robot moving. Update the target position to keep robot continually moveing
   updateSpeed() - sets both motor speeds based on a global vector defined as (X,Y). X determines the speed differential between the wheels, and Y determines the
      direction that the robot should move
   updateSensors() - updates sensor global variables
   updateState() - updates state logic based on sensor value global variables
   randomWanderState() - state for randomly moving the robot as it seeks a wall to follow
   followLeftState() - state for following a wall on the robot's left
   followCenterState() - state for following the middle of a hallway, or between two walls
   followRightState() - state for following a wall on the robot's right
   collideState() - state for dealing with forward collisions (corners and U sections)
   avoidObstacleState() - state for avoiding obstacles when the robot is too close to a wall

   Hardware Connections:
   digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
   digital pin 44 - right stepper motor step pin
   digital pin 49 - right stepper motor direction pin
   digital pin 46 - left stepper motor step pin
   digital pin 53 - left stepper motor direction pin

   digital pin 5 - red LED in series with 220 ohm resistor
   digital pin 6 - green LED in series with 220 ohm resistor
   digital pin 7 - yellow LED in series with 220 ohm resistor

   analog pin 0 - Front IR Sensor
   analog pin 1 - Right IR Sensor
   analog pin 2 - Back IR Sensor
   analog pin 3 - Left IR Sensor

   analog pin 5 - Right Sonar Sensor
   analog pin 6 - Left Sonar Sensor

*/


/************************************#DEFINES*********************************************/

#include <TimerOne.h>       //include library for timers with interrupts
#include <AccelStepper.h>   //include the stepper motor library

// define pin numbers for stepper
#define rtStepPin 44        //right stepper motor step pin
#define rtDirPin 49         //right stepper motor direction pin
#define ltStepPin 46        //left stepper motor step pin
#define ltDirPin 53         //left stepper motor direction pin
#define stepperEnable 48    //stepper enable pin on stepStick 
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

// IR Definitions
#define irB 0       //back IR
#define irL 1       //left IR
#define irF 2       //front IR
#define irR 3       //right IR

// Stepper Library Default Speeds
#define speedD 400          //default speed
#define accelD 800          //default acceleration

// LED Pin Definitions
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states

// Code Operation
#define pauseTime 2500      //time before robot moves

// Sensor reading values
//#define cutoff 9            //cutoff for reading IR sensor values
#define avoidThresh 2         //threshold for avoidObstacle behavior
#define cutoff 10             //cutoff for sensors
#define colThresh 5           //threshold for collide behavior

#define timer_rate 20                  // sensor update calls per second
#define timer_int 1000000/timer_rate   // timer interrupt interval in microseconds

//constants for PD control
#define Kp  0.05      // Kp constant for position control
#define Kd  0.005     // Kd constant for derivative control

// states of the robot
#define randomWander 1    // random wander state
#define followLeft 2      // follow wall on the left state
#define followCenter 3    // follow in the center of walls on both sides state
#define followRight 4     // follow wall on the right state
#define collide 5         // stop the robot from colliding state
#define avoidObstacle 6   // avoid obstacles state


/*******************************GLOBAL VARIABLES*****************************************/

// Variables for converting steps to distances
const int spr = 800;                  //stepper steps per revolution of the wheel
const double d = 3.365;               //diameter of the wheel in inches
const double w = 8.25;                //distance between centers of wheels in inches
const double dstStep = PI * d / spr;  //linear distance of one wheel step
const double l = 8;                   //length of the robot
double X;                             //magnitude of the robot's turning
double Y;                             //magnitude of the robot's movement

// sensor data
double irFront = 20;                  //variable to hold average of current front IR reading
double irBack = 20;                   //variable to hold average of current rear IR reading
double irLeft = 20;                   //variable to hold average of current left IR reading
double irRight = 20;                  //variable to hold average of current right IR reading

// states variables for the robot
byte state = 1;                       //variable to tell what state the robot is in
byte lastState = state;               //variable to tell what the last state the robot was in
unsigned int stateCount = 0;          //variable to count state machine calls
int lostStates = 0;                   //variable to count time since the last state change
int collideCount = 0;                 //variable to count times collide() has been ran in a row

// variables for speed the robot wheels turn
double error = 0;                     //difference that is inputted to controller
double avgDErr = 0;                   //
double lastErr = 0;                   //

int phL = 0;
int phR = 0;

/*******************************INITIALIZE***********************************************/
// Stepper Setup
AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)


/*******************************SETUP****************************************************/
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
  digitalWrite(stepperEnable, stepperEnTrue);     //turns on the stepper motor driver

  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);                   // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updateSensors);           // attaches updateIR() as a timer overflow interrupt

  delay(pauseTime);                               //delay before the robot moves

}

/**************************************LOOP**********************************************/
void loop() {
  drive();

}

/**************************************FUNCTIONS*****************************************/

/*
   drive() function recursively calls itself and makes sure that our drive motors are always moving
   This function also updates the stepper goal position to be the same sign as the current speed, which allows us to
   set the drive direction for the robot using only stepper.setSpeed();
*/
void drive() {
  if (stepperRight.runSpeed()) {              //step the right stepper if a step is required
    stepperRight.move(stepperRight.speed());  //set the right stepper distance equal to the current speed
  }
  if (stepperLeft.runSpeed()) {               //step the left stepper if a step is required
    stepperLeft.move(stepperLeft.speed());    //set the left stepper distance equal to the current speed
  }
  drive();  //recursively call the drive function
}

void fear() {
  Y = 1;
  X = (phL - phR) / 400;
}

void love() {
  Y = 1;
  X = -(phL - phR) / 400;
}

void explorer() {
  Y = 1;
  X = (phL - phR) / 400;
}

void aggressive() {
  Y = 1;
  X = -(phL - phR) / 400;
}





/*
   updateSpeed() updates the speed of both steppers from a global direction vector comprised of the following components:
   X - value in the range {-1, 1} for the direction and magnitude of the wheel speed differential. Positivie values turn right, and
      negative values turn left. X = 0.5 yields a pivot behavior, and abs(X) > 0.5 drives one wheel backwards to increase the possible turn radius.
   Y - value of either 1 or -1 for forward and reverse movement, respectively.
*/
double updateSpeed() {
  int spdL = Y * speedD / 2 + X * speedD; //Calculate the speed of the left stepper using vectors X and Y with range {-1,1}
  int spdR = Y * speedD / 2 - X * speedD; //Calculate the speed of the right stepper using vectors X and Y with range {-1,1}

  stepperLeft.setSpeed(spdL);         //setSpeed for left stepper
  stepperRight.setSpeed(spdR);        //setSpeed for right stepper
  stepperLeft.setMaxSpeed(spdL);      //setMaxSpeed for left stepper
  stepperRight.setMaxSpeed(spdR);     //setMaxSpeed for right stepper
}


/*
   updateSensors() function updates our sensor reading global variables, then calls the updateState() function to
   allow our state machine to update the movement of our robot
*/
void updateSensors() {

  irFront = readIRFront();    //read front IR
  irBack = readIRBack();      //read back IR
  irLeft = readIRLeft();      //read left IR
  irRight = readIRRight();    //read right IR

  updateState();              //update state logic for the robot
}


/*
   updateState() checks several logic conditions in order to determine the new state that the robot should move to.

   The possible states are as follows:
   randomWander = 1;
   followLeft = 2;
   followCenter = 3;
   followRight = 4;
   collide = 5;
   avoidObstacle = 6;
*/
void updateState() {
  stateCount ++;    //increment state counter
  lostStates ++;    //increment lost state counter
  lastErr = error;  //reset the lastErr tracker with the current value of error

  // avoidObstacleState() is called if any of the sensors have a reading below the avoidThresh (2in)
  if (irFront <= avoidThresh || irBack <= avoidThresh || irLeft <= avoidThresh || irRight <= avoidThresh) {
    avoidObstacleState();
    lostStates = 0;
  }
  // collideState() is called if the either the front IR sensor reads below the colThresh or if the state is currently collide.
  // this allows the collide state to conntinue affecting the movement after its initial state conditions are removed.
  else if ( irFront < colThresh || state == collide) {
    collideState();
    lostStates = 0;
  }
  // followCenterState() is called if both the left and right sensors have readings below the cutoff threshold.
  else if (irLeft < cutoff & irRight < cutoff) {
    followCenterState();
    lostStates = 0;
  }
  // followLeftState() is called if the left sensor has a reading below the cutoff threshold.
  else if (irLeft < cutoff) {
    followLeftState();
    lostStates = 0;
  }
  // followRightState() is called if the right sensor has a reading below the cutoff threshold.
  else if (irRight < cutoff) {
    followRightState();
    lostStates = 0;
  }
  // if we have not seen a wall in 50 state cycles, call randomWanderState().
  // if we are already in randomWander, call randomWanderState().
  else if (lostStates > 50 || state == randomWander) {
    randomWanderState();
    lostStates = 0;
  }
  // if we are in followLeft but cannot see the wall, call followLeftState(), but do not reset lostStates;
  else if (state == followLeft) {
    followLeftState();
  }
  // if we are in followRight but cannot see the wall, call followRightState(), but do not reset lostStates;
  else if (state == followRight) {
    followRightState();
  }
  // catchall error state for any combination of inputs we did not expect (used for debugging)
  else {
    setLED("GYR");
  }
}

/************************************STATE*LOGIC***************************************/
/*

*/


/*
   randomWanderState() function creates a random turn (X) vector between -0.3 and 0.3 for the robot to turn randomly. This vector only changes every 2 seconds
*/
void randomWanderState() {
  // led control
  resetLED();
  setLED("R");

  // randomWander movement control
  if (stateCount % 2 * timer_rate == 0) {
    double x = random(-30, 30);   //calculate a random value from -30 to 30
    X = x / 100;                  //scale X to fit within {-0.3, 0.3}
    Y = 1;                        //set Y to forward
    updateSpeed();                //update motor speeds
  }

  // state control
  if (state != randomWander) {
    lastState = state;
    state = randomWander;
  }
}

/*
   followLeftState() function creates a turn vector using PD control based on the error in the robot's position
*/
void followLeftState() {
  // led control
  resetLED();
  setLED("Y");

  // followLeft movement control
  error = irLeft - 5.0;                                   //p error
  avgDErr = (avgDErr * 0.75 + (error - lastErr)) / 1.75;  //d error
  X = Kp * error + Kd * avgDErr;                          //compute X vector using PD control
  Y = 1;    //set Y to forward

  updateSpeed();  //update motor speeds

  // state control
  if (state != followLeft) {
    lastState = state;
    state = followLeft;
  }
}

/*
   followCenterState() function creates a turn vector using PD control based on the error in the robot's position
*/
void followCenterState() {
  // led control
  resetLED();
  setLED("YR");

  error = irLeft - irRight;                               //p error
  avgDErr = (avgDErr * 0.75 + (error - lastErr)) / 1.75;  //d error
  X = Kp * error + Kd * avgDErr;                          //compute X vector using PD control
  Y = 1;    //set Y to forward

  updateSpeed();

  // state control
  if (state != followCenter) {
    lastState = state;
    state = followCenter;
  }
}

/*
   followRightState() function creates a turn vector using PD control based on the error in the robot's position
*/
void followRightState() {
  // led control
  resetLED();
  setLED("G");

  error = 5.0 - irRight;                                  //p error
  avgDErr = (avgDErr * 0.75 + (error - lastErr)) / 1.75;  //d error
  X = Kp * error + Kd * avgDErr;                          //compute X vector using PD control
  Y = 1;    //set Y to forward

  updateSpeed();

  // state control
  if (state != followRight) {
    lastState = state;
    state = followRight;
  }
}

/*
   collideState() function tells the robot to spin to avoid an obstacle when faced with an obstacle in front of the robot.
*/
void collideState() {
  collideCount ++;  //increment counter for collide states.

  // led control
  resetLED();
  setLED("GR");

  //state control
  if (state != collide) {
    lastState = state;
    state = collide;
  }

  //if we were following a wall on the left, spin right
  if (lastState == followLeft || state == followLeft) {
    X = 1;
    Y = 0;
  }
  //default spin left
  else {
    X = -1;
    Y = 0;
  }

  updateSpeed();  //update wheel speeds

  //stay in the collide state while the front sensor is triggered
  if (irFront < colThresh) {
    collideCount = 0;
    state = collide;
  }
  //stay in the collide state for 10 cycles after the front sensor is triggered
  else if (collideCount < 10) {
    state = collide;
  }
  //resume the prior state after leaving collide
  else {
    state = lastState;
    collideCount = 0;
  }
}

/*
   avoidObstacleState() function is called when the robot is too close to an obstacle
   this function drives the robot away from any obstacles it sees in an effort to avoid colliding with them.
*/
void avoidObstacleState() {
  // variable declaration
  double spdL = 0;  //left wheel speed
  double spdR = 0;  //right wheel speed

  // led control
  resetLED();
  setLED("GY");

  // Check which sensors are triggered
  boolean F = (irFront <= avoidThresh);   //check front ir sensor
  boolean B = (irBack  <= avoidThresh);   //check back ir sensor
  boolean R = (irRight <= cutoff);        //check right ir sensor
  boolean L = (irLeft  <= cutoff);        //check left ir sensor

  // Logic to determine the best movement to take
  if (F) {  //if there is an obstacle in front, move backward
    Y = -1;
  }
  if (B) {  //if there is an obstacle behind, move forward
    Y = 1;
  }
  if (R) {  //if there is an obstacle to the right, turn left
    X = -0.5;
  }
  if (L) {  //if there is an obstacle to the left, turn right
    X = 0.5;
  }

  // Set movement speeds on each stepper
  updateSpeed();

  if (state != avoidObstacle) {
    lastState = state;
    state = avoidObstacle;
  }
}



/*
   readIRRight returns the value of the right IR sensor in inches
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
   readIRLeftrns the value of the left IR sensor in inches
*/
double readIRLeft() {
  double A = analogRead(irL);               //read the sensor value
  double val = (2495.0 / (A + 3.0)) - 1.80; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}


/*
   readIRFront returns the value of the front IR sensor in inches
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
   readIRBack returns the value of the back IR sensor in inches
*/
double readIRBack() {
  double A = analogRead(irB);               //read the sensor value
  double val = (1072.0 / (A + 1.0)) - 0.38; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}



/*
   setLED recieves a string including the characters of LEDs that should be turned on.
   Characters include
   'R' --- Red LED
   'Y' --- Yellow LED
   'G' --- Green LED
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
   resetLED turns all three leds to the LOW state.
*/
void resetLED(void) {
  digitalWrite(redLED, LOW);  //reset the red LED
  digitalWrite(ylwLED, LOW);  //reset the yellow LED
  digitalWrite(grnLED, LOW);  //reset the green LED
}

/*
   convert a value from inches/XX to steps/XX
   val is a value with units of inches/XX
*/
int convertStp (double val) {
  int ans = (int) ceil(val / dstStep); //conversion factor
  return ans;   //return ans
}
