/************************************
  Arbib-Lab01.ino
  Peter Garnache and Carson Stone 12/12/19

  This program demonstrates several AI behaviors regarding obstacle avoidance.

  The created functions include the following:
  goToGoal()      - drive to a position while avoiding any encountered obstacles
  avoidObstacle() - avoid obstacles while still moving towards a goal (called as part of the goToGoal program)
  shyKid()        - move away from any obstacles that the robot sees
  angryKid()      - move either forward or backward until the robot senses an obstacle in the way.
  randomWander()  - create a random vector and move the robot in a random direction based on that vector
  smartWander()   - randomly move throughout a room while avoiding any obstacles that the robot encounters

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

#include <AccelStepper.h>   //include the stepper motor library
#include <MultiStepper.h>   //include multiple stepper motor library
#include <NewPing.h>        //include sonar ping library

//define pin numbers
const int rtStepPin = 44;             //right stepper motor step pin
const int rtDirPin = 49;              //right stepper motor direction pin
const int ltStepPin = 46;             //left stepper motor step pin
const int ltDirPin = 53;              //left stepper motor direction pin
const int spr = 800;                  //stepper steps per revolution of the wheel
const int sPinL = A8;                 //left sonar pin
const int sPinR = A9;                 //right sonar pin
const double d = 3.365;               //diameter of the wheel in inches
const double w = 8.25;                //distance between centers of wheels in inches
const double dstStep = PI * d / spr;  //linear distance of one wheel step
const int thresh = 10;                //threshold for stopping (inches)
const int numStep = 5;                //Run the robot constantly for some value steps
const int cutoff = 15;
const int dSpd = 4;                   //default speed for movement
const int turn = 90;                  //default step movement distance for turns
const int straight = 4;               //default movement distance for going forward
char currentCoord = 'X';              //current coordinate that the robot should try to match
int isStop = 0;                       //variable for whether the robot is stopped.
double xPos = 0;                      //x position of the robot
double yPos = 0;                      //y position of the robot
double angle = 0;                     //heading of the robot
double xGoal = 0;                     //x coordinate of the goal
double yGoal = 0;                     //y coordinate of the goal


AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                  //create instance to control multiple steppers at the same time

// Sonar setup
NewPing sonarL(sPinL, sPinL, 200);            //setup left sonar
NewPing sonarR(sPinR, sPinR, 200);            //setup right sonar

#define stepperEnable 48    //stepper enable pin on stepStick 
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define speedD 300          //default speed
#define accelD 1200          //default acceleration

#define irF 0       //front IR
#define irR 1       //right IR
#define irB 2       //back IR
#define irL 3       //left IR


#define pauseTime 2500 //time before robot moves

void setup()
{
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

void loop()
{
  //angryKid(-1);
  //shyKid();
  //randomWander();
  //smartWander();
  Serial.println("begin");
  goToGoal(72, 0);
}

/*
   angryKid() moves in the direction given until it senses an object. It then stops

   dir is a value of either 1 for forward or -1 for backward
*/
void angryKid(int dir) {

  double front = readIRFront(); //read front IR sensor
  double back = readIRBack();   //read back IR sensor
  
  if ( front < thresh || back < thresh) {
    setLED("R");
    stepperLeft.stop();   //stop robot
    stepperRight.stop();  //stop robot
    if (isStop == 0) {
      isStop = 1;
      stepperLeft.setCurrentPosition(0);  //reset the position of the robot to allow for smooth acceleration
      stepperRight.setCurrentPosition(0); //reset the position of the robot to allow for smooth acceleration
    }
  }
  else {
    setLED("G");
    isStop = 0;
    if (!stepperLeft.runSpeed())
      stepperLeft.move(dir * 80000); //if the right stepper reaches its goal position, increase the destination
    if (!stepperRight.runSpeed())
      stepperRight.move(dir * 80000); //if the right stepper reaches its goal position, increase the destination
  }
  resetLED();
}


/*
   shyKid() moves the robot away from any obstacles that it sees with the IR sensors
   It also does not move when the sensors do not see any obstacles
*/
void shyKid() {
  //Create the vector of nearby things to move away from
  double Fr = readIRFront();
  double Ba = readIRBack();
  double Le = readIRLeft();
  double Ri = readIRRight();
  boolean R = Ri < thresh;
  boolean L = Le < thresh;
  boolean B = Ba < thresh;
  boolean F = Fr < thresh;

  // Do not move case, see nothing
  if (!F && !B && !L && !R) {
    resetLED();
  }
  // Do not move case, see something
  else if (F && B && L && R) {
    setLED("Y");
  }
  // Spin left case
  else if (!L && R && F && B || !L && R && !F && !B) {
    setLED("Y");
    spin('L', 5, 2); // spin left 5 degrees at a rate of 2 in/s
  }
  // Spin right case
  else if (F && B || !R && L && !F && !B) { // (F&&B&&!R || !R&&L&&!F&&!B)
    setLED("Y");
    spin('R', 5, 2); // spin right 5 degrees at a rate of 2 in/s
  }
  // Forward case
  else if (!F && B || !F && !B && L && R) {
    setLED("Y");
    drive(1, 2); // move forward for 1 inch at a speed of 2 in/s
  }
  // Backward case
  else if (F && !B) {
    setLED("Y");
    drive(-1, 2); // move backward for 1 inch at a speed of 2 in/s
  }
  else {
    setLED("RYG");
  }
}

/*
   aboidObstacle() is a subfunction of goToGoal that is called whenever goToGoal senses an obstacle.
   This code turns the robot if the front sensor is triggered, and moves straight if one of the side sensors is triggered
*/

void avoidObstacle() {
  //Create the vector of nearby things to move away from
  double Fr = readIRFront();    //read front IR
  double Ba = readIRBack();     //read back IR
  double Le = readIRLeft();     //read left IR
  double Ri = readIRRight();    //read right IR
  boolean R = Ri < thresh;      //logical for right IR sensing object
  boolean L = Le < thresh;      //logical for left IR sensing object
  boolean B = Ba < thresh;      //logical for back IR sensing object
  boolean F = Fr < thresh;      //logical for front IR sensing object

  if (F) {
    setLED("Y");
    if (R) {  //if the robot sees an obstacle in front and on the right, spin left
      spin('L', 90, dSpd);
    }
    else if (L) {   // if the robot sees an obstacle in front and on the left, turn right
      spin('R', 90, dSpd);
    }
    else {    //if the robot sees an obstacle in front and not on the sides, turn based on the direction that gets you closest to the goal
      if (angle == 0 || angle == 180) { // Facing in the x direction
        if (yGoal - yPos > 0) {
          spin('L', 90, dSpd);  //go around the obstacle left if the goal is to the robot's left
        }
        else {
          spin('R', 90, dSpd);  //go around the obstacle right if the goal is to the robot's right
        }
      }
      else { // Facing in the y direction
        if (xGoal - xPos > 0) {
          spin('R', 90, dSpd);  //go around the obstacle left if the goal is to the robot's left
        }
        else {
          spin('L', 90, dSpd);  //go around the obstacle right if the goal is to the robot's right
        }
      }
    }
    avoidObstacle();  //recursively call the function to eliminate while loop
  } else if (!L && !R && !B) {
    //continue back to goToGoal() as we have passed the obstacle
    resetLED();
    drive(straight, dSpd);
    goToGoal();
  }
  else {
    //move straight if the robot senses an obstacle that is not in the front to make sure you clear it before going back to goToGoal()
    setLED("Y");
    drive(straight, dSpd);
    avoidObstacle();
  }
}

/*
   goToGoal() is the first call of the goToGoal function, which sets the xGoal and yGoal positions and then calls goToGoal() without any arguments

   goToGoal takes an x and y coordinate as inputs (in inches)
*/
void goToGoal(int x, int y) {
  xGoal = x;    //set x goal
  yGoal = y;    //set y goal
  goToGoal();
}


/*
   goToGoal() moves the robot forward in the chosen coordinate towards the goal in 1 inch steps.
   If it senses an obstacle in the way, it calls obstacleAvoidance()
*/
void goToGoal() {
  if (xPos == xGoal && yPos == yGoal) {
    setLED("RYG");
  }
  else {
    double Fr = readIRFront();    //read front IR
    double Ba = readIRBack();     //read back IR
    double Le = readIRLeft();     //read left IR
    double Ri = readIRRight();    //read right IR
    boolean R = Ri < thresh;      //logical for right IR sensing object
    boolean L = Le < thresh;      //logical for left IR sensing object
    boolean B = Ba < thresh;      //logical for back IR sensing object
    boolean F = Fr < thresh;      //logical for front IR sensing object
    int theta;                    //init a variable for how far the robot should go

    if (!F && !B && !L && !R) { //if the robot does not sense an obstacle
      boolean doDrive = true;
      if (currentCoord == 'X') {
        if (xPos == xGoal) {    //if the x goal is reached, then set the current coordinate for driving to Y. Don't move for this computational step
          currentCoord = 'Y';
          doDrive = false;
        }
        else {
          if (xGoal - xPos > 0) { //Set the direction that the robot needs to point in to move closer to the goal
            theta = 0;
          }
          else {
            theta = 180;
          }
        }
      }
      else {
        if (yPos == yGoal) { //if the x goal is reached, then set the current coordinate for driving to Y. Don't move for this computational step
          currentCoord = 'Y';
          doDrive = false;
        }
        else {
          if (yGoal - yPos > 0) { //Set the direction that the robot needs to point in to move closer to the goal
            theta = 90;
          }
          else {
            theta = 270;
          }
        }
      }
      if (doDrive) {
        if (angle != theta) {
          goToAngle(dSpd, theta); // if the bot is pointed in the wrong direction for the movement it needs to make, turn it to the right direction
        }
        drive(straight, dSpd);    // assuming the bot is now in the correct position, make a movement forward 1 inch
      }
      goToGoal(); //recursive call to eliminate while loop
    }
    else {
      avoidObstacle(); //if the robot senses something, go to avoiObstacle()
    }
  }
}


/*
   goToAngle calls the spin function and drives the robot to the given heading.
*/
void goToAngle(int spd, int theta) {
  setLED("G");                    //turn on the green led
  spin('L', theta - angle, spd);  //spin to the input angle
  resetLED();                     //reset the leds
}


/*
   smartWander() moves the robot randomly while also avoiding obstacles when sensed.
   If the robot senses an obstacle, it calls shyKid().
   If the robot senses nothing, it calls randomWander().
*/
void smartWander() {
  //Create the vector of nearby things to move away from
  double Fr = readIRFront();    //read front IR
  double Ba = readIRBack();     //read back IR
  double Le = readIRLeft();     //read left IR
  double Ri = readIRRight();    //read right IR
  boolean R = Ri < thresh;      //logical for right IR sensing object
  boolean L = Le < thresh;      //logical for left IR sensing object
  boolean B = Ba < thresh;      //logical for back IR sensing object
  boolean F = Fr < thresh;      //logical for front IR sensing object

  // Random wander case
  if (!F && !B && !L && !R) { //if no obstacles are sensed, randomWander()
    randomWander();
  }
  else {  //if an obstacle is sensed, skyKid();
    shyKid();
  }
  smartWander();  //recursion to eliminate while loop
}


/*
   randomWander() creates a vector of random direction and magnitude, then moves the robot in that direction.
   x and y coordinates
*/
void randomWander() {
  setLED("G");

  double vecX = random(64) - 32; //randomize a x coordinate with 50% chance of being left or right
  double vecY = random(128) - 32; //randomize a y coordinate with 75% chance of being forward
  double mag = sqrt((vecX * vecX) + (vecY * vecY)); //find the magnitude of the vector

  double r_spd = speedD * vecY / mag * (1.0 + vecX / mag);  //choose the left wheel speed to follow the vector
  double l_spd = speedD * vecY / mag * (1.0 - vecX / mag);  //choose the right wheel speed to follow the vector

  stepperLeft.setMaxSpeed(l_spd);   //set the left speed
  stepperRight.setMaxSpeed(r_spd);  //set the right speed

  for (int i = 1; i < 32000; i++) { //move forward at the given speed for a short while
    if (!stepperLeft.runSpeed())
    stepperLeft.move(l_spd);
    if (!stepperRight.runSpeed())
    stepperRight.move(r_spd);
  }

  resetLED();
}


/*
   readIRLeft returns the value of the Left IR sensor in inches using our linearization formula
   we developed in the lab
*/
double readIRLeft() {
  double A = analogRead(irL);               //read the sensor value
  double val = (2421.5 / (A + 1.0)) - 1.92; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}


/*
   readIRRight returns the value of the Right IR sensor in inches using our linearization formula
   we developed in the lab
*/
double readIRRight() {
  double A = analogRead(irR);               //read the sensor value
  double val = (2495.0 / (A + 3.0)) - 1.80; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}


/*
   readIRFront returns the value of the Front IR sensor in inches using our linearization formula
   we developed in the lab
*/
double readIRFront() {
  double A = analogRead(irF);               //read the sensor value
  double val = (1072.0 / (A + 1.0)) - 0.38; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}


/*
   readIRBack returns the value of the Back IR sensor in inches using our linearization formula
   we developed in the lab
*/
double readIRBack() {
  double A = analogRead(irB);               //read the sensor value
  double val = (1049.0 / (A + 7.0)) - 0.23; //use our calculated equations to change from analog to inches
  if (val > cutoff) {
    val = cutoff;                           //eliminate large readings
  }
  return (val);
}


/*
   readSonL returns the value of the Left sonar sensor in inches using our formula developed in
   the lab
*/
double readSonL() {
  unsigned int val = sonarL.ping();
  val = 0.0071 * val - 0.2686;
  if (val > cutoff) {
    val = cutoff;
  }
  return (val);
}


/*
   readSonR returns the value of the Right sonar sensor in inches using our formula developed in
   the lab
*/
double readSonR() {
  unsigned int val = sonarR.ping();
  val = 0.0069 * val - 0.0544;
  if (val > cutoff) {
    val = cutoff;
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
   spin() turns the robot in a spin about the point in the center between its wheels.
*/
void spin(char dir, int theta, int spd) {
  double dist;  //the distance that the robot should mobe in inches
  int neg;      //the direction the right wheel should move
  if (dir == 'L')
    neg = 1;    //move left
  else
    neg = -1;   //move right
  dist = theta * PI / 180.0 * w / 2.0; //calculate the distance to make the theta turn
  spd = convertStp(spd);         //convert speeds from input to steps/sec
  dist = convertStp(dist);       //convert distances from inches to steps
  stepperLeft.move(-neg * dist); //set stepper distance
  stepperRight.move(neg * dist); //set stepper distance
  stepperLeft.setMaxSpeed(spd);  //set stepper speed
  stepperRight.setMaxSpeed(spd); //set stepper speed
  runToStop();                   //move to the desired position
  angle += neg * theta;
}


/*
    drive() moves the robot straight at the speed recieved from its input.
*/
void drive(int dist, int spd) {
  spd = convertStp(spd);          //convert speeds from input to steps/sec]
  xPos += dist * cos(angle * PI / 180.0);
  yPos += dist * sin(angle * PI / 180.0);
  dist = convertStp(dist);        //convert distances from inches to steps
  stepperLeft.move(dist);         //set stepper distance
  stepperRight.move(dist);        //set stepper distance
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
