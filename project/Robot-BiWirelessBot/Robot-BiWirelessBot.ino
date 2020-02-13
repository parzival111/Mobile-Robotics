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

// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define baud_rate 9600

//variables
boolean transmit = false;              //set variable to send or receive data (use same code for both devices but change variable)
RF24 radio(CE_PIN, CSN_PIN);          //create instance of radio object
#define team_channel 123              //set communication channel

const uint64_t pipe = 0xE8E8F0F0E1LL;   //define the radio transmit pipe
byte addresses[][6] = {"1Node", "2Node"};//unique address between transmitter and receiver
uint8_t data[1];                        //variable to hold transmit data
uint8_t memData[36];                    //variable to hold data
uint8_t incoming[1];                    //variable to hold receive data
uint8_t state[] = {0, 0};               //variable to hold receive data position
uint8_t mapDat[4][4];                   //variable to hold receive data MAP
uint8_t path[8];                        //variable to hold path
uint8_t pos[2];                         //variable to hold start positon
uint8_t goal[2];                        //variable to hold goal
uint8_t lastSend;                       // Store last send time
int dataIndex = -1;                     //index of last data recieved

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
}

void loop() {
  if (transmit) {
    ////// Use this code to test sending with the serial port
    readSerial();
    if (data[0] > 0) {
      radio.write(data, sizeof(data));
    }
  }//end of transmit if

  else if (!transmit) {
    /// Use this code to receive from the laptop or the robot
    while (radio.available()) {
      radio.read(&data, sizeof(data));
      if (data[0] > 0) {
        dataIndex++;
        memData[dataIndex] = data[0];
        Serial.println(data[0]);
      }
    }//end while


    if (memData[dataIndex] == 100) {  //data sending is complete
      int i1 = 0, i2 = 0;   //indeces representing the beginning and end of important data streams

      Serial.println("Parsing Data:");

      // Check for path data
      for (int i = i1; i < dataIndex; i++) {
        if (memData[i] == 96) { // we found the end of the path data
          i2 = i;
        }
      }
      for (int i = i1; i <= i2; i++) { // write path data
        path[i] = memData[i];
      }
      i1 = i2 + 1;  // update indeces

      for (int i = i1; i < dataIndex; i++) {
        if (memData[i] == 97) { // we found the end of the position data
          i2 = i;
        }
      }
      if (i2 == i1 + 2) {
        pos[1] = memData[i1];
        pos[2] = memData[i1 + 1];
        Serial.print("Pos: ");
        Serial.print(pos[1]);
        Serial.print(" | ");
        Serial.println(pos[2]);
      }
      i1 = i2 + 1; // update indeces

      for (int i = i1; i < dataIndex; i++) {
        if (memData[i] == 98) { // we found the end of the goal data
          i2 = i;
        }
      }
      if (i2 == i1 + 2) {
        goal[1] = memData[i1];
        goal[2] = memData[i1 + 1];
        Serial.print("Goal: ");
        Serial.print(goal[1]);
        Serial.print(" | ");
        Serial.println(goal[2]);
      }
      i1 = i2 + 1; // update indeces

      for (int i = i1; i < dataIndex; i++) {
        if (memData[i] == 99) { // we found the end of the goal data
          i2 = i;
        }
        if (i2 == i1 + 16) {
          for (int j = 0; j < 4; j++) {
            mapDat[j][1] = memData[i1 + j * 4];
            mapDat[j][2] = memData[i1 + j * 4 + 1];
            mapDat[j][3] = memData[i1 + j * 4 + 2];
            mapDat[j][4] = memData[i1 + j * 4 + 3];
            Serial.print("Map: ");
            Serial.print(mapDat[j][1]);
            Serial.print(" | ");
            Serial.print(mapDat[j][2]);
            Serial.print(" | ");
            Serial.print(mapDat[j][3]);
            Serial.print(" | ");
            Serial.println(mapDat[j][4]);
          }
        }
      }
      transmit = true;
    }
  }

}//end of loop

void readSerial() {
  if (Serial.available() > 0) {
    data[0] = Serial.parseInt();
    if (data[0] > 0) {
      Serial.println(data[0]);
    }
  }
}
