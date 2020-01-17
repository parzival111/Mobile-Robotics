

#include <AccelStepper.h>   //include the stepper motor library
#include <MultiStepper.h>   //include multiple stepper motor library
#include <NewPing.h>        //include sonar ping library
#include <TimerOne.h>




//define pin numbers
const int rtStepPin = 44;             //right stepper motor step pin
const int rtDirPin = 49;              //right stepper motor direction pin
const int ltStepPin = 46;             //left stepper motor step pin
const int ltDirPin = 53;              //left stepper motor direction pin


byte state = 0;
byte flag = 0;


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

#define irB 0       //front IR
#define irR 1       //right IR
#define irF 2       //back IR
#define irL 3       //left IR

#define pauseTime 2500 //time before robot moves

#define timer_int 1000000 // timer interrupt interval in microseconds

///////////// NEW SONAR CLASSES FOR TIMER 2 INTERRUPT/////////////////
//define sonar sensor connections
#define snrLeft   A1   //front left sonar 
#define snrRight  A2  //front right sonar 
#define SONAR_NUM     2         // Number of sensors.
#define MAX_DISTANCE 200        // Maximum distance (in cm) to ping.
#define PING_INTERVAL 125        // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define FIRST_PING_START 50     // First ping starts at this time in ms, gives time for the Arduino to chill before starting.

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(snrLeft, snrLeft, MAX_DISTANCE),//create an instance of the left sonar
  NewPing(snrRight, snrRight, MAX_DISTANCE),//create an instance of the right sonar
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
  delay(pauseTime);

  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);         // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updateSensors);  // attaches updateIR() as a timer overflow interrupt


}

void loop() {
  // put your main code here, to run repeatedly:

}

void updateState() {
  flag = 0;
}

void updateSensor() {
  flag = 1;
  updateIR();
  updateSonar();


}

void updateIR() {
  double Fr = readIRFront();    //read front IR
  double Ba = readIRBack();     //read back IR
  double Le = readIRLeft();     //read left IR
  double Ri = readIRRight();    //read right IR

  //  print IR data
  //    Serial.println("frontIR\tbackIR\tleftIR\trightIR");
  //    Serial.print(irFrontAvg); Serial.print("\t");
  //    Serial.print(irRearAvg); Serial.print("\t");
  //    Serial.print(irLeftAvg); Serial.print("\t");
  //    Serial.println(irRightAvg);
}

/*
  This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
*/
void updateSonar() {
  test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
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
        if (cm[0] > 0)
          srLeftAvg = cm[0];
        if (cm[1] > 0)
          srRightAvg = cm[1];
        Serial.print("lt snr:\t");
        Serial.print(srLeftAvg);
        Serial.print(" cm ");
        Serial.print("\trt snr:\t");
        Serial.print(srRightAvg);
        Serial.println(" cm");
      }
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}

//This function writes to the sonar data if the ping is received
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//This function prints the sonar data once all sonars have been read
void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //Serial.print(i);
    //Serial.print(" = ");
    //Serial.print(cm[i]);
    //Serial.print(" cm\t");
  }
  srLeftAvg = cm[0];
  srRightAvg = cm[1];
  //  Serial.print("Left Sonar = ");
  //  Serial.print(srLeftAvg);
  //  Serial.print("\t\tRight Sonar = ");
  //  Serial.print(srRightAvg);
  //  Serial.println();
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
