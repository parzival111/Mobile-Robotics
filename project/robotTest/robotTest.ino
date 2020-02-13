/*Robot-BiWireless.ino
  Authors: Carlotta Berry
  modified: 02/10/17
  This program will show how to use the Arduino Mega on the robot with the Arduino Uno
  attached to the robot to create bi-directional wireless communication.  This will be used
  to send topological path commands from the laptop to the robot, also to send start and goal positions
  from the laptop to the robot, to receive localization information from the robot to the laptop, and
  to receive a map from the robot to the laptop
  For testing:
    - You can connect both devices to your computer
    - open up 2 instances of the Arduino IDE and put the same program on both
    - upload Mega send code to the robot microcontroller
    - upload Uno receive code on the laptop microcontroller
    - open both serial monitors on your laptop and test the communication

*** HARDWARE CONNECTIONS *****
   https://www.arduino.cc/en/Hacking/PinMapping2560
   Arduino MEGA nRF24L01 connections  *********ROBOT CONNECTION ******************
    CE  pin 7         CSN   pin 8
    VCC 3.3 V         GND GND
    MISO  pin 50      MOSI  pin 51
    SCK pin 52

   http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   http://www.theengineeringprojects.com/2015/07/interfacing-arduino-nrf24l01.html

   Arduino Uno nRF24L01 connections *************LAPTOP CONNECTION **************
   CE  pin 7        CSN   pin 8
   VCC 3.3 V        GND GND
   MOSI pin 11      MISO pin 12
   SCK pin 13
*/

#include <SPI.h>//include serial peripheral interface library
#include <RF24.h>//include wireless transceiver library
#include <nRF24L01.h>//include wireless transceiver library
#include <TimerOne.h>       //include library for timers with interrupts


// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define baud_rate 9600

#define timer_rate 20                   // sensor update calls per second
#define timer_int 1000000/timer_rate   // timer interrupt interval in microseconds
#define sendMessageRate 20

#define mapMapking 1
#define goToGoal 2

#define dataSize 35

//variables
boolean transmit = false;             //set variable to send or receive data (use same code for both devices but change variable)
RF24 radio(CE_PIN, CSN_PIN);          //create instance of radio object
#define team_channel 123              //set communication channel

const uint64_t pipe = 0xE8E8F0F0E1LL;   //define the radio transmit pipe
byte addresses[][6] = {"1Node", "2Node"};//unique address between transmitter and receiver
uint8_t data[1];                        //variable to hold transmit data
uint8_t memData[dataSize];              //variable to hold left over data
uint8_t incoming[1];                    //variable to hold receive data
uint8_t state[] = {0, 0};               //variable to hold receive data position
double irFront = 0;
double irBack = 0;
double irLeft = 0;
double irRight = 0;
uint8_t mapData[4][4];                   //variable to hold receive data MAP
String Path = "";
uint8_t Position[2] = {0, 0};
uint8_t Goal[2] = {0, 0};
double message[23];
uint8_t lastSend;                       // Store last send time
int dataIndex = 0;                      //index of last data recieved
int updateCount = 0;

uint8_t state = 1;
uint8_t followLeft = 0;
uint8_t followRight = 1;


void setup() {
  Serial.begin(baud_rate);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe
    //radio.openWritingPipe(addresses[1]);//open reading pipe
    //radio.openReadingPipe(1, addresses[0]);//open reading pipe
    Serial.println("***********************************");
    Serial.println("....Starting nRF24L01 Transmit.....");
    Serial.println("***********************************");
  } else {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
    //radio.openReadingPipe(1, addresses[1]);//open up reading pipe
    //radio.openWritingPipe(addresses[0]);//open writing pipe
    Serial.println("***********************************");
    Serial.println("....Starting nRF24L01 Receive.....");
    Serial.println("***********************************");
  }

  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);                   // initialize timer1, and set a timer_int second period
  Timer1.attachInterrupt(updateIR);           // attaches updateIR() as a timer overflow interrupt
}

void loop() {

  if (transmit) {

    drive();

  } else if (!transmit) {
    /// Use this code to receive from the laptop or the robot
    while (radio.available()) {
      radio.read(&data, sizeof(data));
      if (data[0] > 0) {
        dataIndex++;
        memData[dataIndex] = data[0];
      }
    }//end while

    if (memData[dataIndex] == 100) {
      transmit = true;
      dataIndex = 0;
      initializeGlobalVar();
    }
  }

}//end of loop

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

void drive() {
  if (stepperRight.runSpeed()) {              //step the right stepper if a step is required
    stepperRight.move(stepperRight.speed());  //set the right stepper distance equal to the current speed
  }
  if (stepperLeft.runSpeed()) {               //step the left stepper if a step is required
    stepperLeft.move(stepperLeft.speed());    //set the left stepper distance equal to the current speed
  }
  drive();  //recursively call the drive function
}


void initializeGlobalVar() {


  irFront = readIRFront();    //read front IR
  irBack = readIRBack();      //read back IR
  irLeft = readIRLeft();      //read left IR
  irRight = readIRRight();    //read right IR

  int ninetyNinePos = positionOf(memData, 99);
  int startval = ninetyNinePos - 16;
  int count = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mapData[i][j] = memData[startval +  count];
      count++;
    }
  }

  int ninetySixPos = positionOf(memData, 96);
  int startVal = ninetSixPos - 2;
  for (int i = 0; i < 2; i++) {
    Path = Path + memData[startval + i];
  }

  int ninetySevenPos = positionOf(memData, 97);
  startval = ninetySevenPos - 2;
  for (int i = 0; i < 2; i++) {
    Position[i] = memData[startval + i];
  }

  int ninetyEightPos = positionOf(memData, 98);
  startval = ninetyEightPos - 2;
  for (int i = 0; i < 2; i++) {
    Goal[i] = memData[startval + i];
  }

}


int positionOf(unit8_t[dataSize] arr, unit8_t val) {
  for (int i = 0; i < arr.length(); i++) {
    if (arr(i) == val) {
      return i;
    }
  }
  return -1;
}

void updateIR() {

  updateCount++;

  if (updateCount > sendMessageRate) {
    sendMessage();
    updateCount = 0;
  }

  irFront = readIRFront();    //read front IR
  irBack = readIRBack();      //read back IR
  irLeft = readIRLeft();      //read left IR
  irRight = readIRRight();    //read right IR
}

void sendMessage() {

  int count = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      message[count] = mapData[i][j];
      count++;
    }
  }
  message[16] = irFront;
  message[17] = irBack;
  message[18] = irLeft;
  message[19] = irRight;
  message[20] = 20;

  for (int i = 0; i <= 20 i++) {
    Serial.println(message[i]);
    readSerial();
    if (data[0] > 0) {
      radio.write(data, sizeof(data));
    }
  }
}

void readSerial() {
  if (Serial.available() > 0) {
    data[0] = Serial.parseInt();
    if (data[0] > 0) {
      Serial.println(data[0]);
    }
  }
}


double updateSpeed() {
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
