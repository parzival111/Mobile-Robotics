/************************************
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
const int rtStepPin = 44;         //right stepper motor step pin 
const int rtDirPin = 49;          //right stepper motor direction pin
const int ltStepPin = 46;         //left stepper motor step pin
const int ltDirPin = 53;          //left stepper motor direction pin
const int spr = 800;              //stepper steps per revolution of the wheel
const double d = 3.365;           //diameter of the wheel in inches
const double w = 8.25;            //distance between centers of wheels in inches
const double dstStep = PI*d/spr;  //linear distance of one wheel step
const double FFL = 1.01;          //fudge factor for the robot's left wheel
const double FFR = 1.01;          //fudge factor for the robot's right wheel

AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                  //create instance to control multiple steppers at the same time

#define stepperEnable 48    //stepper enable pin on stepStick 
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define speedD 1000         //default speed
#define accelD 200          //default acceleration

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
  resetVals();                                  //reset the max speed and acceleration values for each stepper
  steppers.addStepper(stepperRight);            //add right motor to MultiStepper steppers
  steppers.addStepper(stepperLeft);             //add left motor to MultiStepper steppers
  digitalWrite(stepperEnable, stepperEnTrue);   //turns on the stepper motor driver
  delay(pauseTime);                             //delay before the robot moves
}

void loop()
{
pivot('R',90, 1000);
  delay(1000);
  
  /*
   * Example code showing the proper calls for each function
  forward(dist, spd);
  reverse(dist, spd);
  spin(dir, angle, spd);
  pivot(dir, angle, spd);
  turn(dir, angle, spd, r);
  moveCircle(dir, spd, r);
  moveFigure8(dir, spd, r);
  moveSquare(dir, spd, l);
  goToAngle(spd, angle);
  goToGoal(spd, x, y);
  */
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
  double dist;  //the distance that the robot should mobe in inches
  int zL,zR;    //a set of variables that only let one wheel spin
  if(dir == 'L') {
    zL = 1;   //left wheel spins
    zR = 0;   //right wheel does not spin
  }
  else {
    zL = 0;   //left wheel does not spin
    zR = 1;   //right wheel spins
  }
  dist = theta*PI/180.0*w;        //calculate the distance the robot should move, in inches
  int spdL = convertStpL(spd);    //convert speeds from input to steps/sec
  int spdR = convertStpR(spd);    //convert speeds from input to steps/sec
  int distL = convertStpL(dist);  //convert distances from inches to steps
      Serial.print(distL);
  int distR = convertStpR(dist);  //convert distances from inches to steps
  stepperLeft.move(zL*distL);     //set stepper distance
  stepperRight.move(zR*distR);    //set stepper distance
  stepperLeft.setMaxSpeed(spdL);  //set stepper speed
  stepperRight.setMaxSpeed(spdR); //set stepper speed
  runToStop();                    //move to the desired position
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
  double dist;  //the distance that the robot should mobe in inches
  int neg;      //the direction the right wheel should move
  if(dir == 'L')
    neg = 1;    //move left
  else
    neg = -1;   //move right
  dist = theta*PI/180.0*w/2.0;    //

  int spdL = convertStpL(spd);    //convert speeds from input to steps/sec
  int spdR = convertStpR(spd);    //convert speeds from input to steps/sec
  int distL = convertStpL(dist);  //convert distances from inches to steps
  int distR = convertStpR(dist);  //convert distances from inches to steps
  stepperLeft.move(-neg*distL);   //set stepper distance
  stepperRight.move(neg*distR);   //set stepper distance
  stepperLeft.setMaxSpeed(spdL);  //set stepper speed
  stepperRight.setMaxSpeed(spdR); //set stepper speed
  runToStop();                    //move to the desired position
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
void turn(char dir, int theta, int spd, int r) {
  long positions[2];                          //create a positions vector for each stepper
  stepperLeft.setCurrentPosition(0);          //reset the stepper position
  stepperRight.setCurrentPosition(0);         //reset the steper position
  stepperLeft.setMaxSpeed(convertStpL(spd));  //set stepper max speeds
  stepperRight.setMaxSpeed(convertStpL(spd)); //set stepper max speeds
  if(dir == 'L'){
    positions[0] = convertStpL(theta * PI/180.0*(r + w/2.0)); //set the left stepper target position
    positions[1] = convertStpR(theta * PI/180.0*(r - w/2.0)); //set the right stepper target position
  }
  else {
    positions[0] = convertStpL(theta * PI/180.0*(r - w/2.0)); //set the left stepper target position
    positions[1] = convertStpR(theta * PI/180.0*(r + w/2.0)); //set the right stepper target position
  }
  steppers.moveTo(positions);     //set each stepper's position
  steppers.runSpeedToPosition();  //move stepers to position at constant speed

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
  int spdL = convertStpL(spd);    //convert speeds from input to steps/sec
  int spdR = convertStpR(spd);    //convert speeds from input to steps/sec
  int distL = convertStpL(dist);  //convert distances from inches to steps
  int distR = convertStpR(dist);  //convert distances from inches to steps
  stepperLeft.move(distL);        //set stepper distance
  stepperRight.move(distR);       //set stepper distance
  stepperLeft.setMaxSpeed(spdL);  //set stepper speed
  stepperRight.setMaxSpeed(spdR); //set stepper speed
  runToStop();                    //move to the desired position
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
  int spdL = convertStpL(spd);    //convert speeds from input to steps/sec
  int spdR = convertStpR(spd);    //convert speeds from input to steps/sec
  int distL = convertStpL(dist);  //convert distances from inches to steps
  int distR = convertStpR(dist);  //convert distances from inches to steps
  stepperLeft.move(-distL);       //set stepper distance
  stepperRight.move(-distR);      //set stepper distance
  stepperLeft.setMaxSpeed(spdR);  //set stepper speed
  stepperRight.setMaxSpeed(spdL); //set stepper speed
  runToStop();                    //move to the desired position
}


/*
  stop has no inputs.

  Stop calls the function left.stop() and right.stop() in order to stop the robot.
*/
void Stop() {
  stepperLeft.stop();   //stop steppers
  stepperRight.stop();  //stop steppers
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
  setLED("R");              //set red LED
  turn(dir, 360, spd, r);   //turn in a circle with dir = dir, for 360 degrees, at spd = spd, and r = r
  resetLED();               //reset leds after movement
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
  setLED("RY");               //turn on red and yellow LEDs
  if(dir=='L'){
    turn('L', 360, spd, r);   //turn left 360 deg
    delay(100);
    turn('R', 360, spd, r);   //turn right 360 deg
  }
  else{
    turn('R', 360, spd, r);   //turn right 360 deg
    delay(100);
    turn('L', 360, spd, r);   //turn left 360 deg
  }
  resetLED();                 //reset LEDs
} 


/*
  moveSquare takes 3 inputs; a character representation of the direction of the initial turn,
  an integer for the speed that the turn and movements should be completed at, and a value for
  the length for the sides of the square in inches.

  dir has a value of either ‘L’ or ‘R’ 
  spd is a value from 100 to 1000 in steps/sec
  l is a value from 0 to 100 in inches

  In a for loop that iterates 4 times, the forward function is called with the inputted length
  and speed and then the pivot function is called to turn 90 degrees using the inputted speed
  and direction.

 */
void moveSquare(char dir, int spd, int l) {
  setLED("RYG");    //turn on the red, yellow, and green LEDs
  for(int i = 0 ; i < 4; i++) {
    forward(l, spd);      //move forward for the length of the square
    spin(dir, 90, spd);   //spin 90 degrees in the indicated direction
  }
  resetLED();             //reset the LEDs
}


/*
  goToAngle takes 2 inputs; an integer for the speed that the turn should be completed at, and
  a value for the angle the robot will spin to.

  spd is the speed of the robot ranging from 100 to 1000 in steps/sec
  theta is the angle that the robot should move to

  This function just calls the spin command and makes the robot end at a specific angle
  relative to the original direction the robot was facing before the function was called

 */
void goToAngle(int spd, int theta) {
  setLED("G");            //turn on the green led
  spin('L', theta, spd);  //spin to the input angle
  resetLED();             //reset the leds
}


/*
  goToGoal takes 3 inputs; an integer for the speed that the movement should be completed at,
  and a x and y position for the robot to go to.

  spd is the speed of the robot ranging from 100 to 1000 in steps/sec
  l is a value from 0 to 100 in inches

  This function uses atan2(x, y) to find the angle required to position the robot pointed
  towards the goal. The robot then uses the spin command to rotate to that angle. The robot
  uses the Pythagorean theorem to calculate the distance it should travel to end at the goal.
  It then moves forward to that position. 

 */
void goToGoal(int spd, double x, double y) {
  setLED("GY");                           //turn on the green and yellow LEDs
  double theta = atan2(y, x) * 180.0/PI;  //calculate the angle to move to
  goToAngle(spd, theta);                  //go to the angle
  setLED("GY");                           //turn on the green and yellow LEDs
  double dist = sqrt(sq(x) + sq(y));      //calculate the distance to go to
  forward(dist, spd);                     //move forward the distance
  resetLED();                             //reset the leds
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
  convert a value from inches/XX to steps/XX for the left wheel

  val is a value with units of inches/XX
 */
int convertStpL (double val) {
  int ans;      //temp variable
  ans = (int) ceil(val/dstStep*FFL);  //conversion factor
  return ans;   //return ans
}


/*
  convert a value from inches/XX to steps/XX or the right wheel

  val is a value with units of inches/XX
 */
int convertStpR (double val) {
  int ans;      //temp variable
  ans = (int) ceil(val/dstStep*FFR);  //conversion factor
  return ans;   //return ans
}


/*
  setLED recieves a string including the characters of LEDs that should be turned on.
  Characters include
  'R' --- Red LED
  'Y' --- Yellow LED
  'G' --- Green LED
 */
void setLED(String led){
  resetLED();                   //reset the LEDs
  if(led.indexOf('R') >= 0){    //if the string input contains the character 'R'
    digitalWrite(redLED, HIGH); //set the red LED to HIGH
  }
  if(led.indexOf('Y') >= 0){    //if the string input contains the character 'Y'
    digitalWrite(ylwLED, HIGH); //set the yellow LED to HIGH
  }
  if(led.indexOf('G') >= 0){    //if the string input contains the character 'G'
    digitalWrite(grnLED, HIGH); //set the green LED to HIGH
  }
}


/*
  resetLED turns all three leds to the LOW state.
 */
void resetLED(void){
  digitalWrite(redLED, LOW);  //reset the red LED
  digitalWrite(ylwLED, LOW);  //reset the yellow LED
  digitalWrite(grnLED, LOW);  //reset the green LED
}


/*
  resetVals resets the acceleration and maxSpeed values for each stepper motor
 */
void resetVals() {
  stepperRight.setMaxSpeed(speedD);               //set the maximum speed for the right stepper
  stepperLeft.setMaxSpeed(speedD);                //set the maximum speed for the left stepper
  stepperRight.setAcceleration(accelD);           //set the initial acceleration for the right stepper
  stepperLeft.setAcceleration(accelD);            //set the initial acceleration for the right stepper
}
