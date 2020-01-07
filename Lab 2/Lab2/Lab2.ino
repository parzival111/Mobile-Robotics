/************************************
  Arbib-Lab01.ino
  Peter Garnache 12/05/19

  This program demonstrates several AI behaviors regarding obstacle avoidance. 
  
  
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
const int rtStepPin = 44;         //right stepper motor step pin 
const int rtDirPin = 49;          //right stepper motor direction pin
const int ltStepPin = 46;         //left stepper motor step pin
const int ltDirPin = 53;          //left stepper motor direction pin
const int spr = 800;              //stepper steps per revolution of the wheel
const int sPinL = A8;             //left sonar pin
const int sPinR = A9;             //right sonar pin
const double d = 3.365;           //diameter of the wheel in inches
const double w = 8.25;            //distance between centers of wheels in inches
const double dstStep = PI*d/spr;  //linear distance of one wheel step
int thresh = 10;                  //threshold for stopping (inches)
int numStep = 5;                  //Run the robot constantly for some value steps
int isStop = 0;                   //variable for whether the robot is stopped.
int cutoff = 15;

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
  steppers.addStepper(stepperRight);            //add right motor to MultiStepper steppers
  steppers.addStepper(stepperLeft);             //add left motor to MultiStepper steppers
  digitalWrite(stepperEnable, stepperEnTrue);   //turns on the stepper motor driver
  delay(pauseTime);                             //delay before the robot moves
}

void loop()
{
  //angryKid(-1);
  shyKid();
  //randomWander();
  //smartWander();
}

/*
 * 
 * 
 */
void angryKid(int dir){

  double front = readIRFront();
  double back = readIRBack();
  if( front < thresh || back < thresh){
    setLED("R");
    stepperLeft.stop();
    stepperRight.stop();
    if(isStop == 0){
      isStop = 1;
      stepperLeft.setCurrentPosition(0);
      stepperRight.setCurrentPosition(0);
    }
  }
  else{
    setLED("G");
    isStop = 0;
    if(!stepperLeft.runSpeed())
    stepperLeft.move(dir*80000);
    if(!stepperRight.runSpeed())
    stepperRight.move(dir*80000);
  }
  resetLED();
}


/*
 * 
 * 
 */
void shyKid(){
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
  if(!F&&!B&&!L&&!R){
    resetLED();
  }
  // Do not move case, see something
  else if(F&&B&&L&&R){
    setLED("Y");
  }
  // Spin left case
  else if(!L&&R&&F&&B || !L&&R&&!F&&!B){
    setLED("Y");
    spin('L', 5, 2);
  }
  // Spin right case
  else if(F&&B || !R&&L&&!F&&!B){ // (F&&B&&!R || !R&&L&&!F&&!B)
    setLED("Y");
    spin('R', 5, 2);
  }
  // Forward case
  else if(!F&&B || !F&&!B&&L&&R){
    setLED("Y");
    drive(1, 2);
  }
  // Backward case
  else if(F&&!B){
    setLED("Y");
    drive(-1, 2);
  }
  else{
    setLED("RYG");
  }
}

/*
 * 
 * 
 */
void smartWander(){
  //Create the vector of nearby things to move away from
  double Fr = readIRFront();
  double Ba = readIRBack();
  double Le = readIRLeft();
  double Ri = readIRRight();
  boolean R = Ri < thresh;
  boolean L = Le < thresh;
  boolean B = Ba < thresh;
  boolean F = Fr < thresh;
  
  // Random wander case
  if(!F&&!B&&!L&&!R){
    randomWander();
  }
  else{
    shyKid();
  }
}

 
void randomWander(){
  //Create the vector of nearby things to move away from
  setLED("G");
  
  double vecX = random(128)-32;
  double vecY = random(128)-32;
  double mag = sqrt((vecX*vecX)+(vecY*vecY));
  
  double r_spd = speedD*vecY/mag*(1.0 + vecX/mag);
  double l_spd = speedD*vecY/mag*(1.0 - vecX/mag);
  if (mag < 5){
    stepperLeft.stop();
    stepperRight.stop();
    if(isStop == 0){
      isStop = 1;
      stepperLeft.setCurrentPosition(0);
      stepperRight.setCurrentPosition(0);
    }
  }
  else{   
    stepperLeft.setMaxSpeed(l_spd);
    stepperRight.setMaxSpeed(r_spd);
    
    for(int i = 1; i < 32000; i++){
      if(!stepperLeft.runSpeed());
      stepperLeft.move(l_spd);
      if(!stepperRight.runSpeed());
      stepperRight.move(r_spd);
    }
  }
  resetLED();
}

 

/*
 * readIRLeft returns the value of the Left IR sensor in inches using our linearization formula
 * we developed in the lab
 */
double readIRLeft(){
  double A = analogRead(irL);
  double val = (2421.5/(A+1.0))-1.92;
  if(val > cutoff){
    val = cutoff;
  }
  return(val);
}

/*
 * readIRRight returns the value of the Right IR sensor in inches using our linearization formula
 * we developed in the lab
 */
double readIRRight(){
  double A = analogRead(irR);
  double val = (2495.0/(A+3.0))-1.80;
  if(val > cutoff){
    val = cutoff;
  }
  return(val);
}

/*
 * readIRFront returns the value of the Front IR sensor in inches using our linearization formula
 * we developed in the lab
 */
double readIRFront(){
  double A = analogRead(irF);
  double val = (1072.0/(A+1.0))-0.38;
  if(val > cutoff){
    val = cutoff;
  }
  return(val);
}

/*
 * readIRBack returns the value of the Back IR sensor in inches using our linearization formula
 * we developed in the lab
 */
double readIRBack(){
  double A = analogRead(irB);
  double val = (1049.0/(A+7.0))-0.23;
  if(val > cutoff){
    val = cutoff;
  }
  return(val);
}


/*
 * readSonL returns the value of the Left sonar sensor in inches using our formula developed in 
 * the lab
 */
double readSonL(){
  unsigned int val = sonarL.ping();
  val = 0.0071*val-0.2686;
  if(val > cutoff){
    val = cutoff;
  }
  return(val);
}


/*
 * readSonR returns the value of the Right sonar sensor in inches using our formula developed in 
 * the lab
 */
double readSonR(){
  unsigned int val = sonarR.ping();
  val = 0.0069*val-0.0544;
  if(val > cutoff){
    val = cutoff;
  }
  return(val);
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
 * spin() turns the robot in a spin about the point in the center between its wheels. 
*/
void spin(char dir, int theta, int spd) {
  double dist;  //the distance that the robot should mobe in inches
  int neg;      //the direction the right wheel should move
  if(dir == 'L')
    neg = 1;    //move left
  else
    neg = -1;   //move right
  dist = theta*PI/180.0*w/2.0;   //calculate the distance to make the theta turn
  spd = convertStp(spd);         //convert speeds from input to steps/sec
  dist = convertStp(dist);       //convert distances from inches to steps
  stepperLeft.move(-neg*dist);   //set stepper distance
  stepperRight.move(neg*dist);   //set stepper distance
  stepperLeft.setMaxSpeed(spd);  //set stepper speed
  stepperRight.setMaxSpeed(spd); //set stepper speed
  runToStop();                   //move to the desired position
}


/* 
 *  drive() moves the robot straight at the speed recieved from its input. 
*/
void drive(int dist, int spd) {
  spd = convertStp(spd);          //convert speeds from input to steps/sec]
  dist = convertStp(dist);        //convert distances from inches to steps
  stepperLeft.move(dist);         //set stepper distance
  stepperRight.move(dist);        //set stepper distance
  stepperLeft.setMaxSpeed(spd);   //set stepper speed
  stepperRight.setMaxSpeed(spd);  //set stepper speed
  runToStop();                    //move to the desired position
}


/*
 * runToStop runs both the right and left stepper until they stop moving
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
  int ans = (int) ceil(val/dstStep);  //conversion factor
  return ans;   //return ans
}



/*
 * Vector movement behavior originally created for shyKid, useful for smartWander. 
double vecX = Le - Ri;
  double vecY = Fr - Ba;
  double mag = sqrt((vecX*vecX)+(vecY*vecY));
  if(vecY == 0){
    vecY = mag;
  }
  double r_spd = speedD*vecY/mag*(1.0 + vecX/mag);
  double l_spd = speedD*vecY/mag*(1.0 - vecX/mag);
  if (mag < 5){
    stepperLeft.stop();
    stepperRight.stop();
    if(isStop == 0){
      isStop = 1;
      stepperLeft.setCurrentPosition(0);
      stepperRight.setCurrentPosition(0);
    }
  }
  else{   
    stepperLeft.setMaxSpeed(l_spd);
    stepperRight.setMaxSpeed(r_spd);
    
    for(int i = 1; i < 999; i++){
      if(!stepperLeft.runSpeed());
      stepperLeft.move(l_spd);
      if(!stepperRight.runSpeed());
      stepperRight.move(r_spd);
    }
  }
  */
