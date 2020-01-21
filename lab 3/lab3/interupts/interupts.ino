
#include <AccelStepper.h>   //include the stepper motor library
#include <MultiStepper.h>   //include multiple stepper motor library
#include <NewPing.h>        //include sonar ping library
#include <TimerOne.h>       //include library for timers with interrupts

////////////////////GLOBAL VARIABLES////////////////////////////////

// define pin numbers
const int rtStepPin = 44;             //right stepper motor step pin
const int rtDirPin = 49;              //right stepper motor direction pin
const int ltStepPin = 46;             //left stepper motor step pin
const int ltDirPin = 53;              //left stepper motor direction pin
const int snrLeft = A8;               //left sonar pin
const int snrRight = A9;              //right sonar pin

// constant numbers other methods use for movement
const int numStep = 5;                //Run the robot constantly for some value steps
const int dSpd = 4;                   //default speed for movement
const int turn = 90;                  //default step movement distance for turns
const int straight = 3;               //default movement distance for going forward
const double bangAngle = 15;          //angle to turn for bangBang()

// sensor data
double irFront = 20;                  //variable to hold average of current front IR reading
double irBack = 20;                   //variable to hold average of current rear IR reading
double irLeft = 20;                   //variable to hold average of current left IR reading
double irRight = 20;                  //variable to hold average of current right IR reading
double srLeft = 20;                   //variable to hold average of left sonar current reading
double srRight = 20;                  //variable to hold average or right sonar current reading

// values based off of dimentions of robot
const int spr = 800;                  //stepper steps per revolution of the wheel
const double d = 3.365;               //diameter of the wheel in inches
const double w = 8.25;                //distance between centers of wheels in inches
const double dstStep = PI * d / spr;  //linear distance of one wheel step

// logic to run methods
const int thresh = 5;                 //threshold for stopping (inches)
const int cutoff = 20;                // max value for sonar
int isStop = 0;                       //variable for whether the robot is stopped
int count = 0;                        //value to see number of times it does not see a wall for bangBang to abort back into drive straight
char currentCoord = 'X';              //current coordinate that the robot should try to match
byte flag = 0;                        //flag to let the state know the interupt has been triggered

// position tracking
double xPos = 0;                      //x position of the robot
double yPos = 0;                      //y position of the robot
double angle = 0;                     //heading of the robot
double xGoal = 0;                     //x coordinate of the goal
double yGoal = 0;                     //y coordinate of the goal

// states for the robot
byte state = 0;                       //variable to tell what state the robot is in
byte nothing = 1;
byte front = 2;
byte back = 3;
byte left = 4;
byte right = 5;
byte leftSonar = 6;
byte rightSonar = 7;

//////////////////////////////////////////////////////////////////////////////////

AccelStepper stepperLeft(AccelStepper::DRIVER, rtStepPin, rtDirPin);    //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperRight(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                  //create instance to control multiple steppers at the same time

////////////////////////////////////////////////////////////////////////////////////

#define stepperEnable 48    //stepper enable pin on stepStick 
#define redLED 5            //red LED for displaying states
#define ylwLED 6            //green LED for displaying states
#define grnLED 7            //yellow LED for displaying states
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define speedD 300          //default speed
#define accelD 1200          //default acceleration

#define irB 0       //front IR
#define irR 1       //right IR
#define irF 2       //back IR
#define irL 3       //left IR

#define pauseTime 2500 //time before robot moves

#define timer_int 200000 // timer interrupt interval in microseconds

///////////// NEW SONAR CLASSES FOR TIMER 2 INTERRUPT/////////////////

//define sonar sensor connections
#define SONAR_NUM     2                     // Number of sensors.
#define MAX_DISTANCE 200                     // Maximum distance(in) to ping.
#define PING_INTERVAL 500                   // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define FIRST_PING_START 100                // First ping starts at this time in ms, gives time for the Arduino to chill before starting.

unsigned long pingTimer[SONAR_NUM];         // Holds the times when the next ping should happen for each sensor.
unsigned int pingDistances[SONAR_NUM];      // Where the ping distances are stored.
uint8_t currentSensor = 0;                  // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {                // Sensor object array. first object is left sonar. second object is right sonar
  NewPing (snrLeft, snrLeft, MAX_DISTANCE),   //create an instance of the left sonar
  NewPing (snrRight, snrRight, MAX_DISTANCE), //create an instance of the right sonar
};
////////////////////////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:

  //multipler sonar on timer 2 setup
  pingTimer[0] = millis() + FIRST_PING_START;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++)               // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

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

  stepperLeft.setSpeed(3000);        //set left motor speed
  stepperRight.setSpeed(3000);      //set right motor speed

  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);                   // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updateSensor);           // attaches updateIR() as a timer overflow interrupt

  delay(pauseTime);


}

void loop() {
  if (flag = 1) {
    updateState();
  }

  stepperRight.runSpeed();
  stepperLeft.runSpeed();


}

void updateState() {
  flag = 0;
  //Serial.println(irFront);
  //Serial.println(thresh);
  if (irFront < thresh) {
    frontState();

  } else if (irBack < thresh) {
    backState();

  } else if (irLeft < thresh) {
    leftState();

  } else if (irRight < thresh) {
    rightState();

  } else if (srLeft < thresh) {
    leftSonarState();

  } else if (srRight < thresh) {
    rightSonarState();

  } else {
    nothingState();
  }
}


void nothingState() {
  resetLED();
  setLED("R");
  state = nothing;
}
void frontState() {
  resetLED();
  setLED("Y");
  state = front;
}
void backState() {
  resetLED();
  setLED("YR");
  state = back;
}
void leftState() {
  resetLED();
  setLED("G");
  state = left;
}
void rightState() {
  resetLED();
  setLED("GR");
  state = right;
}
void leftSonarState() {
  resetLED();
  setLED("GY");
  state = leftSonar;
}
void rightSonarState() {
  resetLED();
  setLED("GYR");
  state = rightSonar;
}


void updateSensor() {
  //Serial.println("check");
  flag = 1;
  updateIR();
  updateSonar();
}


void updateIR() {
  irFront = readIRFront();    //read front IR
  irBack = readIRBack();     //read back IR
  irLeft = readIRLeft();     //read left IR
  irRight = readIRRight();    //read right IR

  //  print IR data
  //    Serial.println("frontIR\tbackIR\tleftIR\trightIR");
  //    Serial.print(irFront); Serial.print("\t");
  //    Serial.print(irRear); Serial.print("\t");
  //    Serial.print(irLeft); Serial.print("\t");
  //    Serial.println(irRight);
}

/*
  This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
*/
void updateSonar() {
  //test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  //digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    //    Serial.print("\t\t\t");
    //    Serial.print(millis());
    //    Serial.print("\t");
    //    Serial.print(pingTimer[i]);
    //    Serial.print("\t");
    //    Serial.println(PING_INTERVAL);
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) {
        //oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
        if (pingDistances[0] > 0)
          srLeft = pingDistances[0];
        if (pingDistances[1] > 0)
          srRight = pingDistances[1];
        //Serial.print("lt snr:\t");
        //Serial.print(srLeft);
        //Serial.print(" pingDistances ");
        //Serial.print("\trt snr:\t");
        //Serial.print(srRight);
        //Serial.println(" pingDistances");
      }
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      pingDistances[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}

//This function writes to the sonar data if the ping is received
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()) {

    pingDistances[currentSensor] = (sonar[currentSensor].ping_result/ US_ROUNDTRIP_CM)*0.3937;


  }
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
double convertSonL(int leftSonarVal) {
  double inches = 0.0071 * leftSonarVal - 0.2686;
  if (inches > cutoff) {
    inches = cutoff;
  }
  return (inches);
}


/*
   readSonR returns the value of the Right sonar sensor in inches using our formula developed in
   the lab
*/
double convertSonR(int rightSonarVal) {
  double inches = 0.0069 * rightSonarVal - 0.0544;
  if (inches > cutoff) {
    inches = cutoff;
  }
  return (inches);
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
    neg = -1;    //move left
  else
    neg = 1;   //move right
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


//This function prints the sonar data once all sonars have been read
void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  // for (uint8_t i = 0; i < SONAR_NUM; i++) {
  //Serial.print(i);
  //Serial.print(" = ");
  //Serial.print(cm[i]);
  //Serial.print(" cm\t");
  //}
  //srLeft = pingDistances[0];
  //srRight = pingDistances[1];
  //  Serial.print("Left Sonar = ");
  //  Serial.print(srLeft);
  //  Serial.print("\t\tRight Sonar = ");
  //  Serial.print(srRight);
  //  Serial.println();
}
