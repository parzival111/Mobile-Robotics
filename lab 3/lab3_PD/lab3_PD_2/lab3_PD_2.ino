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
#define speedD 300         //default speed
#define accelD 1200         //default acceleration
#define updateDist 12800    //

// LED Pin Definitions
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states

// Code Operation
#define pauseTime 2500      //time before robot moves

// Sensor reading values
#define cutoff 9           //cutoff for reading IR sensor values
#define avoidThresh 2

#define timer_int 100000    // timer interrupt interval in microseconds
#define timer_rate = 1/(timer_int*0.000001);

//constants for PD control
#define Kp  0.1
//#define Kd  0.1

// states of the robot
#define randomWander 1
#define followLeft 2
#define followCenter 3
#define followRight 4
#define collide 5
#define avoidObstacle 6


/*******************************GLOBAL VARIABLES*****************************************/
// Variables for converting steps to distances

const int spr = 800;                  //stepper steps per revolution of the wheel
const double d = 3.365;               //diameter of the wheel in inches
const double w = 8.25;                //distance between centers of wheels in inches
const double dstStep = PI * d / spr;  //linear distance of one wheel step
const double l = 8;                   //length of the robot
double r;                             //radius of curvature the robot is turning

// sensor data
double irFront = 20;                  //variable to hold average of current front IR reading
double irBack = 20;                   //variable to hold average of current rear IR reading
double irLeft = 20;                   //variable to hold average of current left IR reading
double irRight = 20;                  //variable to hold average of current right IR reading

// states variables for the robot
byte state = 0;                       //variable to tell what state the robot is in

// variables for speed the robot wheels durn
double error = 0;                      //diference that is inputted to controller
//double derror = 0;                     //rate of change of deference between current and desired


/*******************************INITIALIZE***********************************************/
// Stepper Setup
AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)


/*******************************SETUP***********************************************/
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

/*******************************LOOP***********************************************/
void loop() {
  drive();

}

/*
   drive() function recurcively calls itself and makes sure that our drive motors are always moving
*/
void drive() {
  if (stepperRight.runSpeed()) {
    stepperRight.move(stepperRight.speed());
  }
  if (stepperLeft.runSpeed()) {
    stepperLeft.move(stepperLeft.speed());
  }
  drive();
}



void updateSensors() {

  irFront = readIRFront();    //read front IR
  irBack = readIRBack();     //read back IR
  irLeft = readIRLeft();     //read left IR
  irRight = readIRRight();    //read right IR

  updateState();

  //  print IR data
  //    Serial.println("frontIR\tbackIR\tleftIR\trightIR");
  //    Serial.print(irFront); Serial.print("\t");
  //    Serial.print(irRear); Serial.print("\t");
  //    Serial.print(irLeft); Serial.print("\t");
  //    Serial.println(irRight);
}


/*
  randomWander = 1;
  followLeft = 2;
  followCenter = 3;
  followRight = 4;
  collide = 5;
  avoidObstacle = 6;
*/
void updateState() {

  if (irFront <= avoidThresh || irBack <= avoidThresh || irLeft <= avoidThresh || irRight <= avoidThresh) {
    avoidObstacleState();

  } else if ( irFront < cutoff) {
    collideState();

  } else if (irLeft < cutoff && irRight < cutoff) {
    followCenterState();

  } else if (irLeft < cutoff) {
    followLeftState();

  } else if (irRight < cutoff) {
    followRightState();

  } else if (irFront >= cutoff && irLeft >= cutoff && irRight >= cutoff) {
    randomWanderState();

  } else {   // catch in case we do not go to a defined state
    resetLED();
    setLED("GYR");
  }

}

void randomWanderState() {
  resetLED();
  setLED("R");
  // give the radius the chance to change using the random() function
  r = random(25) - 12;
  updateSpeed();
  state = randomWander;
}

void followLeftState() {
  resetLED();
  setLED("Y");

  error = irLeft - 5.0;
  r = 1 / (Kp * error);

  updateSpeed();

  state = followLeft;
}

void followCenterState() {
  resetLED();
  setLED("YR");

  error = irRight - irLeft;
  r = 1 / (Kp * error);

  updateSpeed();

  state = followCenter;


}

void followRightState() {
  resetLED();
  setLED("G");

  error = 5.0 - irRight;
  r = 1 / (Kp * error);

  updateSpeed();

  state = followRight;
}

void collideState() {
  resetLED();
  setLED("GR");

  if (state == followLeft) {
    error = (cutoff / front);
    r = 1 / (Kp * error);

    updateSpeed();
    
  } else if (state == followRight) {
    
    error = (cutoff / front);
    r = 1 / (Kp * error);

    updateSpeed();

  } else if (state == followCenter) {

    state = followLeft

    error = (cutoff / front);
    r = 1/(Kp * error);

    updateSpeed();
  }

}

void avoidObstacleState() {
  double spdL = 0;  //left wheel speed
  double spdR = 0;  //right wheel speed

  // LED logic
  resetLED();
  setLED("GY");

  // Check which sensors are triggered
  boolean F = (irFront <= avoidThresh);   //check front ir sensor
  boolean B = (irBack  <= avoidThresh);   //check back ir sensor
  boolean R = (irRight <= avoidThresh);   //check right ir sensor
  boolean L = (irLeft  <= avoidThresh);   //check left ir sensor

  // Logic to determine the best movement to take
  if (F) {
    spdL = spdR - speedD / 2;
    spdR = spdL - speedD / 2;
  }
  if (B) {
    spdL = spdL + speedD / 2;
    spdR = spdR + speedD / 2;
  }
  if (R) {
    spdL = spdL - speedD / 2;
    spdR = spdR + speedD / 2;
  }
  if (L) {
    spdL = spdL + speedD / 2;
    spdR = spdR - speedD / 2;
  }

  // Set movement speeds on each stepper
  stepperLeft.setSpeed(spdL);
  stepperRight.setSpeed(spdR);

  state = avoidObstacle;
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


double updateSpeed() {
  if (abs(r) >=  150) {
    stepperLeft.setSpeed(speedD * (r / abs(r)));
    stepperRight.setSpeed(speedD * (r / abs(r)));
  }

  //  } else if (r <= 5) {
  //    stepperLeft.setSpeed = speedD * (-r / abs(r));
  //    stepperRight.setSpeed = speedD * (r / abs(r));
  //  }

  stepperLeft.setSpeed(speedD * (abs(r - (w / 2)) / abs(r)));
  stepperRight.setSpeed(speedD * (abs(r + (w / 2)) / abs(r)));
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
