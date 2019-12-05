/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  Arbib-Lab01.ino
  Peter Garnache 12/05/19

  This program demonstrates several primary functions necessary for controlling a mobile robot.
  The primary functions created are
  moveCircle - Move the robot in a complete circle with a specified radius
  moveFigure8 - Move the robot in a figure eight pattern with a specified radius
  forward, reverse - Move the robot a specified distance either forward or backward
  pivot- Turn the robot with the axis of rotation being one of the wheels
  spin - Turn the robot with the axis of rotation being between the two wheels
  turn - Turn the robot at a variable radius where both wheels move
  stop - Stop the robot in place during a move
  
  Hardware Connections:
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 44 - right stepper motor step pin
  digital pin 49 - right stepper motor direction pin
  digital pin 46 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor
*/

#include <AccelStepper.h>   //include the stepper motor library
#include <MultiStepper.h>   //include multiple stepper motor library

//define pin numbers
const int rtStepPin = 44; //right stepper motor step pin 
const int rtDirPin = 49;  //right stepper motor direction pin
const int ltStepPin = 46; //left stepper motor step pin
const int ltDirPin = 53;  //left stepper motor direction pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);   //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);    //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                  //create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick 
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

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
  stepperRight.setMaxSpeed(1500);               //set the maximum speed for the right stepper
  stepperLeft.setMaxSpeed(1500);                //set the maximum speed for the left stepper
  stepperRight.setAcceleration(3000);           //set the initial acceleration for the right stepper
  stepperLeft.setAcceleration(3000);            //set the initial acceleration for the right stepper
  steppers.addStepper(stepperRight);            //add right motor to MultiStepper steppers
  steppers.addStepper(stepperLeft);             //add left motor to MultiStepper steppers
  digitalWrite(stepperEnable, stepperEnTrue);   //turns on the stepper motor driver
  delay(pauseTime);                             //Delay before the robot moves
}

void loop()
{
  
}


/*
  pivot takes three inputs: a character representation of direction, an integer for the
  angle rotation to be completed, and an integer for the speed that the turn should be
  completed at.

  dir has a value of either ‘L’ or ‘R’ 
  theta is a value from -3600 to 3600 in degrees
  spd is a value from 100 to 1000 in steps/sec

  Pivot will be using the run() command to drive the robot through a turn using
  acceleration. This allows us to improve precision by using the acceleration option in
  the stepper library to eliminate skipped steps and wheel skid during the start and end
  of movement. We will determine the number of steps the wheel must travel using
  kinematics. We will set the distance for the stepper to move to using stepper.move(),
  then use a loop with stepper.run() to complete the spin. We will choose the stepper to
  run based on the “dir” variable.
*/
void pivot(char dir, int theta, int spd) {
  
}


/*
  spin takes three inputs: a character representation of direction, an integer for the
  angle rotation to be completed, and an integer for the speed that the turn should be
  completed at.

  dir has a value of either ‘L’ or ‘R’ 
  theta is a value from -3600 to 3600 in degrees
  spd is a value from 100 to 1000 in steps/sec

  Spin will be using the run() command to drive the robot through a turn using
  acceleration. This allows us to improve precision by using the acceleration option in
  the stepper library to eliminate skipped steps and wheel skid during the start and end
  of movement. We will determine the number of steps the wheel must travel using 
  kinematics. We will set the distance for each stepper to move to using stepper.move(),
  then use a loop with stepper.run() on each side to complete the spin. One wheel will
  be given a positive distance to move, and the other will be given a negative distance
  to move. 
*/
void spin(char dir, int theta, int spd) {
  
}


/*
  Turn takes four inputs: a character representation of direction, an integer for the
  angle rotation to be completed, an integer for the speed that the turn should be
  completed at, and a value for the radius of the circle the center of the robot will
  follow in inches.

  dir has a value of either ‘L’ or ‘R’ 
  theta is a value from -3600 to 3600 in degrees
  spd is a value from 100 to 1000 in steps/sec
  r is a value from 0 to 100 in inches

  Turn will be using the run() command to drive the robot through a turn using
  acceleration. This allows us to improve precision by using the acceleration option in
  the stepper library to eliminate skipped steps and wheel skid during the start and end
  of movement. We will determine the number of steps the wheel must travel using
  kinematics. We can also calculate the speeds that each wheel must travel at to achieve
  the desired average speed of the center of the robot, and turn the robot at the desired
  radius. Because we are using acceleration, and each wheel is spinning at different
  speeds, we will need to use kinematics equations for constant acceleration systems to
  solve for the acceleration needed by each wheel so that they accelerate over the same
  time period, so that the ratio of speeds from the outer wheel to the inner wheel is
  always the right ratio for the radius of turn that we want.  We will set the distance
  for each stepper to move to using stepper.move(), then use a loop with stepper.run()
  on each side to complete the turn.

*/
void turn(char dir, int theta, int spd) {
  
}


/*
  forward has two inputs: an integer for the distance to travel in inches, and an 
  integer for the speed of travel.
  
  dist is a value from -1000 to 1000 in inches
  spd is a value from 100 to 1000 in steps/sec

  The function then sets the desired position using stepper.move(dist) on both inputs. 
  It then runs a while loop that waits until both steppers complete their movement that 
  calls left.run() and right.run() each loop. This allows us to improve precision by 
  using the acceleration option in the stepper library to eliminate skipped steps and 
  wheel skid during the start and end of movement. 
*/
void forward(int dist, int spd) {
  
}


/*
  reverse has two inputs: an integer for the distance to travel in inches, and an 
  integer for the speed of travel.

  dist is a value from -1000 to 1000 in inches
  spd is a value from 100 to 1000 in steps/sec

  The function then sets the desired position using stepper.move(-dist) on both inputs.
  It then runs a while loop that waits until both steppers complete their movement that
  calls left.run() and right.run() each loop. This allows us to improve precision using
  the acceleration option in the stepper library to eliminate skipped steps and wheel
  skid during the start and end of movement. 
*/
void reverse(int dist, int spd) {
  
}


/*
  stop has no inputs.

  Stop calls the function left.stop() and right.stop() in order to stop the robot.
*/
void stop() {
  stepperLeft.stop();
  stepperRight.stop();
}


/*
  moveCircle takes 3 inputs: a character representation of direction, an integer for
  the speed that the turn should be completed at, and a value for the radius of the
  circle the center of the robot will follow in inches.

  dir has a value of either ‘L’ or ‘R’ 
  spd is a value from 100 to 1000 in steps/sec
  r is a value from 0 to 100 in inches

  This function will call the turn() function with the following call:
  turn(dir, 360, spd, r);

*/
void moveCircle(char dir, int spd, int r) {
  
}


/*
  moveFigure8 takes 3 inputs: a character representation of direction of the initial
  turn, an integer for the speed that the turn should be completed at, and a value for
  the radius of the circle the center of the robot will follow in inches.

  dir has a value of either ‘L’ or ‘R’ 
  spd is a value from 100 to 1000 in steps/sec
  r is a value from 0 to 100 in inches

  moveFigure uses the moveCircle command to create a figure 8. It calls it twice with
  the “dir” input changing based on the input to moveFigure8. We will include this
  change in the if loops that check the dir variable character. 
  For a “dir” input of ‘L’, the calls would be as follows:
  moveCircle(L, spd, r);
  moveCircle(R, spd, r);
*/
void moveFigure8(char dir, int spd, int r) {
  
}
