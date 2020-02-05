/************************************
   Arbib-Lab03.ino
   Peter Garnache and Carson Stone 02/02/2020

   This program demonstrates the 4 Bratenburg vehicles

   The created functions include the following:
   fearState() - If the robot sees a light on the left then the left motor moves faster
   loveState() - If the robot sees a light on the left then the left motor goes slower
   explorerState() - If the robot sees a light on the left then the right motor goes faster
   aggressiveState() - If the robot sees a light on the left then the right motor goes slower

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

   analog pin 4 - Right photo resistor
   analog pin 5 - Left photo resistor
*/

/************************************#DEFINES*********************************************/

#include <TimerOne.h>       //include library for timers with interrupts
#include <AccelStepper.h>   //include the stepper motor library

// define pin numbers for stepper
#define rtStepPin 44        //right stepper motor step pin
#define rtDirPin 49         //right stepper motor direction pin
#define ltStepPin 46        //left stepper motor step pin
#define ltDirPin 53         //left stepper motor direction pin
#define lfPhotoResist A4
#define rtPhotoResist A5
#define stepperEnable 48    //stepper enable pin on stepStick 
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

// IR Definitions
#define irB A0       //back IR
#define irL A1       //left IR
#define irF A2       //front IR
#define irR A3       //right IR

// Stepper Library Default Speeds
#define speedD 400          //default speed
#define accelD 4000          //default acceleration

// LED Pin Definitions
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states

// Code Operation
#define pauseTime 500      //time before robot moves

// Sensor reading values
#define avoidThresh 6         //threshold for avoidObstacle behavior
#define cutoff 10             //cutoff for sensors

#define timer_rate 20                  // sensor update calls per second
#define timer_int 1000000/timer_rate   // timer interrupt interval in microseconds

//constants for PD control
#define Kp  0.05      // Kp constant for position control
#define Kd  0.005     // Kd constant for derivative control

// states of the robot
#define randomWander 1    // random wander state
#define followLeft 2      // follow wall on the left state
#define followRight 3     // follow wall on the right state
#define followLight 4     // followLight state
#define Fear 2            // fear state
#define Love 3            // love state
#define Aggressive 4      // aggressive state
#define Explorer 5        // explorer state
#define avoidObstacle 5   // avoid obstacles state

#define lightThresh 850   // light threshold


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

double photoLeft = 0;                 //varaible holding the value of the left photoresistor
double photoRight = 0;                //variable holding the value of the right photoresistor

double mode = Explorer;               //current mode for the robot's behavior

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
  pinMode(lfPhotoResist, INPUT);                  //sets pin as input
  pinMode(rtPhotoResist, INPUT);                  //sets pin as input

  // Stepper initialization
  stepperRight.setMaxSpeed(speedD);               //set the maximum speed for the right stepper
  stepperLeft.setMaxSpeed(speedD);                //set the maximum speed for the left stepper
  stepperRight.setAcceleration(accelD);           //set the initial acceleration for the right stepper
  stepperLeft.setAcceleration(accelD);            //set the initial acceleration for the right stepper
  digitalWrite(stepperEnable, stepperEnTrue);     //turns on the stepper motor driver

  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);                   // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updateSensors);          // attaches updateIR() as a timer overflow interrupt

  delay(pauseTime);                               //delay before the robot moves

}

/**************************************LOOP**********************************************/
void loop() {
  drive();    //continually call the drive() function to keep the robot moving
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

/*
   updateSpeed() updates the speed of both steppers from a global direction vector comprised of the following components:
   X - value in the range {-1, 1} for the direction and magnitude of the wheel speed differential. Positivie values turn right, and
      negative values turn left. X = 0.5 yields a pivot behavior, and abs(X) > 0.5 drives one wheel backwards to increase the possible turn radius.
   Y - value of either 1 or -1 for forward and reverse movement, respectively.
*/
double updateSpeed() {
  int spdL = Y * speedD + X * speedD; //Calculate the speed of the left stepper using vectors X and Y with range {-1,1}
  int spdR = Y * speedD - X * speedD; //Calculate the speed of the right stepper using vectors X and Y with range {-1,1}

  stepperLeft.setSpeed(spdL);         //setSpeed for left stepper
  stepperRight.setSpeed(spdR);        //setSpeed for right stepper
  stepperLeft.setMaxSpeed(abs(spdL));      //setMaxSpeed for left stepper
  stepperRight.setMaxSpeed(abs(spdR));     //setMaxSpeed for right stepper
}

/*
   updateSensors() function updates our sensor reading global variables, then calls the updateState() function to
   allow our state machine to update the movement of our robot
*/
void updateSensors() {

  photoLeft = analogRead(lfPhotoResist);  //update left photoresistor
  if (photoLeft < lightThresh) {
    photoLeft = lightThresh;
  }

  photoRight = analogRead(rtPhotoResist); //update right photoresistor
  if (photoRight < lightThresh) {
    photoRight = lightThresh;
  }

  irFront = readIRFront();    //read front IR
  irBack = readIRBack();      //read back IR
  irLeft = readIRLeft();      //read left IR
  irRight = readIRRight();    //read right IR

  updateState();              //update state logic for the robot
}

/*
   updateState() checks several logic conditions in order to determine the new state that the robot should move to.

   The possible states are as follows:
   Fear 2
   Love 3
   Aggressive 4
   Explorer 5
   avoidObstacle 6
*/
void updateState() {
  stateCount ++;    //increment state counter
  resetLED();       //reset LEDs

  // avoidObstacleState() is called if any of the sensors have a reading below the avoidThresh (2in)
  if (irFront <= avoidThresh || irBack <= avoidThresh || irLeft <= avoidThresh || irRight <= avoidThresh) {
    avoidObstacleState();
    setLED("R");
  }

  //check to see whether the robot senses a light
  else if (photoLeft > lightThresh || photoRight > lightThresh) {
    if (mode == Fear) {
      fearState();        //call fear state
      setLED("RYG");      //LEDs
    }
    else if (mode == Love) {
      loveState();        //call love state
      setLED("RY");       //LEDs
    }
    else if (mode == Aggressive) {
      aggressiveState();  //call aggressive state
      setLED("YG");       //LEDs
    }
    else if (mode == Explorer) {
      explorerState();    //call explorer state
      setLED("RG");       //LEDs
    }
  }

  //call randomWander if the robot does not see light or a wall
  else {
    randomWanderState();
    setLED("G");
  }
}

/*
   randomWanderState() function creates a random turn (X) vector between -0.3 and 0.3 for the robot to turn randomly. This vector only changes every 2 seconds
*/
void randomWanderState() {
  // randomWander movement control
  if (stateCount % (2 * timer_rate) == 0) {
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
   If the robot sees a light on the left, then the left motor moves faster
*/
void fearState() {
  Y = 1;
  X = (photoLeft - photoRight) / 400;
  updateSpeed();                //update motor speeds
}

/*
   If the robot sees a light on the left, then the left motor goes slower
*/
void loveState() {
  Y = 1;
  X = -(photoLeft - photoRight) / 400;
  updateSpeed();                //update motor speeds
}

/*
   If the robot sees a light on the left, then the right motor goes faster
*/
void explorerState() {
  Y = 1;
  X = (photoLeft - photoRight) / 400;
  updateSpeed();                //update motor speeds
}

/*
   If the robot sees a light on the left, then the right motor goes slower
*/
void aggressiveState() {
  Y = 1;
  X = -(photoLeft - photoRight) / 400;
  updateSpeed();                //update motor speeds
}


/*
   avoidObstacleState() function is called when the robot is too close to an obstacle
   this function drives the robot away from any obstacles it sees in an effort to avoid colliding with them.
*/
void avoidObstacleState() {
  // variable declaration
  double spdL = 0;  //left wheel speed
  double spdR = 0;  //right wheel speed


  // Check which sensors are triggered
  boolean F = (irFront < avoidThresh);   //check front ir sensor
  boolean B = (irBack  < avoidThresh);   //check back ir sensor
  boolean R = (irRight < cutoff);        //check right ir sensor
  boolean L = (irLeft  < cutoff);        //check left ir sensor

  // Logic to determine the best movement to take
  if (F) {  //if there is an obstacle in front, move backward
    Y = -1;
  }
  if (B) {  //if there is an obstacle behind, move forward
    Y = 1;
  }
  if (R || !L && !R) {  //if there is an obstacle to the right, turn left
    X = 0.5 * Y;
  }
  if (L) {  //if there is an obstacle to the left, turn right
    X = -0.25 * Y;
  }

  // Set movement speeds on each stepper
  updateSpeed();

  // state logic
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
