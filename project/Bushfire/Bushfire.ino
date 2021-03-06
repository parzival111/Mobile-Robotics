
#include <SPI.h>//include serial peripheral interface library
#include <RF24.h>//include wireless transceiver library
#include <nRF24L01.h>//include wireless transceiver library
#include <TimerOne.h>       //include library for timers with interrupts
#include <AccelStepper.h>   //include the stepper motor library

#define timer_rate 10                  // sensor update calls per second
#define timer_int 1000000/timer_rate   // timer interrupt interval in microseconds

// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define baud_rate 9600

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
#define speedD 800          //default speed
#define accelD 1600         //default acceleration

// LED Pin Definitions
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states

// Code Operation
#define pauseTime 0500      //time before robot moves

#define timer_rate 20                  // sensor update calls per second
#define timer_int 1000000/timer_rate   // timer interrupt interval in microseconds

//constants for PD control
#define Kp  0.05      // Kp constant for position control
#define Kd  0.005     // Kd constant for derivative control

/*******************************GLOBAL VARIABLES*****************************************/

// Variables for converting steps to distances
const int spr = 800;                  //stepper steps per revolution of the wheel
const double d = 3.365;               //diameter of the wheel in inches
const double w = 8.25;                //distance between centers of wheels in inches
const double dstStep = PI * d / spr;  //linear distance of one wheel step
const double l = 8;                   //length of the robot
double X;                             //magnitude of the robot's turning
double Y;                             //magnitude of the robot's movement

uint8_t mapData[4][4];
uint8_t bushMap[4][4] =  {
  {16, 16, 16, 16},
  {16, 16, 16, 16},
  {16, 16, 16, 16},
  {16, 16, 16, 16}
};

uint8_t pos[2] = {0, 0};
uint8_t goal[2] = {0, 0};

int counter = 0;

// sensor data
double irFront = 20;                  //variable to hold average of current front IR reading
double irBack = 20;                   //variable to hold average of current rear IR reading
double irLeft = 20;                   //variable to hold average of current left IR reading
double irRight = 20;                  //variable to hold average of current right IR reading

double cutOff = 9;

// states variables for the robot
byte state = 1;                       //variable to tell what state the robot is in

boolean isMoving = false;             //variable to keep track of non-continuous movement commands

// variables for speed the robot wheels turn
double error = 0;                     //difference that is inputted to controller
double avgDErr = 0;                   //
double lastErr = 0;                   //

boolean doDecide = false;

int rowPos;
int colPos;

int Direction;



/*******************************INITIALIZE***********************************************/
// Stepper Setup
AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // open the serial port at 9600 baud


  //map 1

  mapData[0][0] = 16;
  mapData[0][1] = 2;
  mapData[0][2] = 8;
  mapData[0][3] = 12;
  mapData[1][0] = 10;
  mapData[1][1] = 7;
  mapData[1][2] = 10;
  mapData[1][3] = 7;
  mapData[2][0] = 9;
  mapData[2][1] = 6;
  mapData[2][2] = 3;
  mapData[2][3] = 12;
  mapData[3][0] = 15;
  mapData[3][1] = 14;
  mapData[3][2] = 5;
  mapData[3][3] = 7;

  // map 2
  /*
    mapData[0][0] = 12;
    mapData[0][1] = 16;
    mapData[0][2] = 16;
    mapData[0][3] = 12;
    mapData[1][0] = 9;
    mapData[1][1] = 6;
    mapData[1][2] = 6;
    mapData[1][3] = 3;
    mapData[2][0] = 11;
    mapData[2][1] = 16;
    mapData[2][2] = 16;
    mapData[2][3] = 11;
    mapData[3][0] = 13;
    mapData[3][1] = 8;
    mapData[3][2] = 16;
    mapData[3][3] = 15;
  */

  /*******************************SETUP***********************************************/

  //starting row
  pos[0] = 0;

  //starting column
  pos[1] = 2;

  //ending row
  goal[0] = 2;

  //ending column
  goal[1] = 3;

  //starting direction
  Direction = 4;

  /******************************************************************************/



  bushMap[goal[0]][goal[1]] = counter;

  rowPos = pos[0];
  colPos = pos[1];



  // put your setup code here, to run once:
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

  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);

  // solve the grassfire expansion for the robot
  for (int i = 0; i < 11; i++) {
    expand();
  }

  /*
    // solve the grassfire expansion for the robot
    for (int i = 0; i < 5; i++) {
      expandPath();
    }

  */

  delay(pauseTime);//delay before the robot moves
  decide();
}

void loop() {
  // put your main code here, to run repeatedly
  if (doDecide) {
    decide();
  }
  else if (isMoving == false) {
    drive();
  }
}


void drive() {
  if (!stepperRight.runSpeed()) {              //step the right stepper if a step is required
    stepperRight.move(stepperRight.speed());  //set the right stepper distance equal to the current speed
  }
  if (!stepperLeft.runSpeed()) {               //step the left stepper if a step is required
    stepperLeft.move(stepperLeft.speed());    //set the left stepper distance equal to the current speed
  }
}

void updateSensors() {
  irFront = readIRFront();    //read front IR
  irBack = readIRBack();      //read back IR
  irLeft = readIRLeft();      //read left IR
  irRight = readIRRight();    //read right IR

  if (!isMoving) {
    updateState();
    double steps = (stepperRight.currentPosition() + stepperLeft.currentPosition()) / 2.0;
    if (steps > convertStp(18)) {
      doDecide = true;              //update state logic for the robot
    }
  }
}

void decide() {
  double val = bushMap[rowPos][colPos];

  if ((colPos < 3) && (bushMap[rowPos][colPos + 1] == val - 1) && (sensorDirection(2) > cutOff)) {
    //Serial.print("#1 |");
    //Serial.println(Direction);
    spin('R', 90 * (2 - Direction));
    Direction = 2;
    colPos = colPos + 1;
  }
  else if ((rowPos < 3) && (bushMap[rowPos + 1][colPos] == val - 1) && (sensorDirection(3) > cutOff)) {
    //Serial.print("#2 |");
    //Serial.println(Direction);
    spin('R', 90 * (3 - Direction));
    Direction = 3;
    rowPos = rowPos + 1;
  }
  else if ((colPos > 0) && (bushMap[rowPos][colPos - 1] == val - 1) && (sensorDirection(4) > cutOff)) {
    //Serial.print("#3 |");
    //Serial.println(Direction);
    spin('R', 90 * (4 - Direction));
    Direction = 4;
    colPos = colPos - 1;
  }
  else if ((rowPos > 0) && (bushMap[rowPos - 1][colPos] == val - 1) && (sensorDirection(1) > cutOff)) {
    //Serial.print("#4 |");
    //Serial.println(Direction);
    spin('R', 90 * (1 - Direction));
    Direction = 1;
    rowPos = rowPos - 1;
  }
  else {
    while (1);
  }
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  doDecide = false;
}

double sensorDirection(int side) {
  double val = 0;

  if (Direction == 1) {

    switch (side) {
      case 1:
        return irFront;
        break;
      case 2:
        return irRight;
        break;
      case 3:
        return irBack;
        break;
      case 4:
        return irLeft;
        break;
      default:
        return 0;
        break;
    }
  }

  else if (Direction == 2) {

    switch (side) {
      case 1:
        return irLeft;
        break;
      case 2:
        return irFront;
        break;
      case 3:
        return irRight;
        break;
      case 4:
        return irBack;
        break;
      default:
        return 0;
        break;
    }
  }
  else if (Direction == 3) {

    switch (side) {
      case 1:
        return irBack;
        break;
      case 2:
        return irLeft;
        break;
      case 3:
        return irFront;
        break;
      case 4:
        return irRight;
        break;
      default:
        return 0;
        break;
    }
  }
  else if (Direction == 4) {

    switch (side) {
      case 1:
        return irRight;
        break;
      case 2:
        return irBack;
        break;
      case 3:
        return irLeft;
        break;
      case 4:
        return irFront;
        break;
      default:
        return 0;
        break;
    }
  }
  else {
    return 0;
  }
}


void updateState() {
  if (irFront < 2.5) {
    doDecide = true;
  }

  if ((irLeft < cutOff && irRight < cutOff)) {
    followCenterState();
  }
  else if (irLeft < cutOff) {
    followLeftState();
  }
  else if (irRight < cutOff) {
    followRightState();
  }
  else {
    X = 0;
    Y = 1;
    updateSpeed();
  }
}


void followLeftState() {
  // led control
  resetLED();
  setLED("Y");

  // followLeft movement control
  error = irLeft - 5;                   //p error
  X = Kp * error;                         //compute X vector using PD control
  Y = 1;    //set Y to forward

  updateSpeed();  //update motor speeds

}

void followCenterState() {
  // led control
  resetLED();
  setLED("YR");

  error = irLeft - irRight;               //p error
  X = Kp * error;                         //compute X vector using PD control
  Y = 1;    //set Y to forward

  updateSpeed();
}

void followRightState() {
  // led control
  resetLED();
  setLED("G");

  error = 5 - irRight;                  //p error
  X = Kp * error;                         //compute X vector using PD control
  Y = 1;    //set Y to forward

  updateSpeed();
}

void updateSpeed() {
  int spdL = Y * speedD / 2 + X * speedD; //Calculate the speed of the left stepper using vectors X and Y with range {-1,1}
  int spdR = Y * speedD / 2 - X * speedD; //Calculate the speed of the right stepper using vectors X and Y with range {-1,1}

  stepperLeft.setSpeed(spdL);         //setSpeed for left stepper
  stepperRight.setSpeed(spdR);        //setSpeed for right stepper
  stepperLeft.setMaxSpeed(spdL);      //setMaxSpeed for left stepper
  stepperRight.setMaxSpeed(spdR);     //setMaxSpeed for right stepper
}

/*
   readIRRight returns the value of the right IR sensor in inches
*/
double readIRRight() {
  double A = analogRead(irR);               //read the sensor value
  double val = (2421.5 / (A + 1.0)) - 1.92; //use our calculated equations to change from analog to inches
  if (val > 20) {
    val = 20;                           //eliminate large readings
  }
  return (val);
}


/*
   readIRLeftrns the value of the left IR sensor in inches
*/
double readIRLeft() {
  double A = analogRead(irL);               //read the sensor value
  double val = (2495.0 / (A + 3.0)) - 1.80; //use our calculated equations to change from analog to inches
  if (val > 20) {
    val = 20;                           //eliminate large readings
  }
  return (val);
}


/*
   readIRFront returns the value of the front IR sensor in inches
*/
double readIRFront() {
  double A = analogRead(irF);               //read the sensor value
  double val = (1049.0 / (A + 7.0)) - 0.23; //use our calculated equations to change from analog to inches
  if (val > 20) {
    val = 20;                           //eliminate large readings
  }
  return (val);
}

/*
   readIRBack returns the value of the back IR sensor in inches
*/
double readIRBack() {
  double A = analogRead(irB);               //read the sensor value
  double val = (1072.0 / (A + 1.0)) - 0.38; //use our calculated equations to change from analog to inches
  if (val > 20) {
    val = 20;                           //eliminate large readings
  }
  return (val);
}

void displayMap() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      Serial.print("   ");
      Serial.print(mapData[row][col]);
      if (row == pos[0] && col == pos[1]) {
        Serial.print("P  ");
      }
      else if (row == goal[0] && col == goal[1]) {
        Serial.print("G  ");
      } else {
        Serial.print("   ");
      }
    }
    Serial.println();
    Serial.println();
  }

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
}

void displayBushMap() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      Serial.print("   ");
      Serial.print(bushMap[row][col]);
      Serial.print("   ");
    }
    Serial.println();
    Serial.println();
  }
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
}

void expand() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {

      if (bushMap[row][col] == counter) {

        switch (mapData[row][col]) {

          case 1:
            topOpen(row, col);
            rightOpen(row, col);
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 2:
            rightOpen(row, col);
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 3:
            topOpen(row, col);
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 4:
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 5:
            topOpen(row, col);
            rightOpen(row, col);
            leftOpen(row, col);
            break;
          case 6:
            rightOpen(row, col);
            leftOpen(row, col);
            break;
          case 7:
            topOpen(row, col);
            leftOpen(row, col);
            break;
          case 8:
            leftOpen(row, col);
            break;
          case 9:
            topOpen(row, col);
            rightOpen(row, col);
            bottomOpen(row, col);
            break;
          case 10:
            rightOpen(row, col);
            bottomOpen(row, col);
            break;
          case 11:
            topOpen(row, col);
            bottomOpen(row, col);
            break;
          case 12:
            bottomOpen(row, col);
            break;
          case 13:
            topOpen(row, col);
            rightOpen(row, col);
            break;
          case 14:
            rightOpen(row, col);
            break;
          case 15:
            topOpen(row, col);
            break;
          case 16:

            break;
          default:

            break;

        }

      }

    }

  }
  counter++;
  displayMap();
  displayBushMap();
}



void topOpen(int row, int col) {
  if (row != 0) {
    if (bushMap[row - 1][col] == 16) {
      bushMap[row - 1][col] = counter + 1;
    }
  }
}

void rightOpen(int row, int col) {
  if (col != 3) {
    if (bushMap[row][col + 1] == 16) {
      bushMap[row][col + 1] = counter + 1;
    }
  }
}

void bottomOpen(int row, int col) {
  if (row != 3) {
    if (bushMap[row + 1][col] == 16) {
      bushMap[row + 1][col] = counter + 1;
    }
  }
}

void leftOpen(int row, int col) {
  if (row != 0) {
    if (bushMap[row][col - 1] == 16) {
      bushMap[row][col - 1] = counter + 1;
    }
  }
}

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

void spin(char dir, int theta) {
  double dist;  //the distance that the robot should move in inches
  int neg;      //the direction the right wheel should move
  if (dir == 'L')
    neg = -1;    //move left
  else
    neg = 1;   //move right
  dist = theta * PI / 180.0 * w / 2.0;  //calculate the distance to make the theta turn
  dist = convertStp(dist);              //convert distances from inches to steps
  stepperLeft.move(-neg * dist);        //set stepper distance
  stepperRight.move(neg * dist);        //set stepper distance
  stepperLeft.setMaxSpeed(speedD / 2);    //set stepper speed
  stepperRight.setMaxSpeed(speedD / 2);   //set stepper speed
  runToStop();                          //move to the desired position
}

/*
   convert a value from inches/XX to steps/XX
   val is a value with units of inches/XX
*/
int convertStp (double val) {
  int ans = (int) ceil(val / dstStep); //conversion factor
  return ans;   //return ans
}


void runToStop ( void ) {
  int runL = 1;   //state variabels
  int runR = 1;   //state variables
  isMoving = true;
  while (runL || runR) {        //until both stop
    if (!stepperRight.run()) {  //step the right stepper, if it is done moving set runR = 0
      runR = 0;                 //right done moving
    }
    if (!stepperLeft.run()) {   //step the left stepper, if it is done moving set runL = 0
      runL = 0;                 //left done moving
    }
  }
  isMoving = false;
}
