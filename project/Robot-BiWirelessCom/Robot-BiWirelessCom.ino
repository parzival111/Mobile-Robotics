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
boolean transmit = true;             //set variable to send or receive data (use same code for both devices but change variable)
RF24 radio(CE_PIN, CSN_PIN);          //create instance of radio object
#define team_channel 123              //set communication channel

const uint64_t pipe = 0xE8E8F0F0E1LL;   //define the radio transmit pipe
byte addresses[][6] = {"1Node", "2Node"};//unique address between transmitter and receiver
uint8_t data[1];                        //variable to hold transmit data
uint8_t memData[24];                    //variable to hold left over data
uint8_t incoming[1];                    //variable to hold receive data
uint8_t state[] = {0, 0};               //variable to hold receive data position
uint8_t mapDat[4][4];                   //variable to hold receive data MAP
uint8_t lastSend;                       // Store last send time]
int dataIndex = 0;                      //index of last data

void setup() {
  Serial.begin(baud_rate);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  radio.openWritingPipe(pipe);//open up writing pipe
}

void loop() {
  if (transmit) {
    readSerial();
    if (data[0] > 0) {
      radio.write(data, sizeof(data));
    }
  }
  else if (!transmit) {
    /// Use this code to receive from the laptop or the robot
    while (radio.available()) {
      radio.read(&data, sizeof(data));
      if (data[0] > 0) {
        Serial.println(data[0]);
        dataIndex++;
        memData[dataIndex] = data[0];
      }
    }//end while
  }


  // last digit 99 means we just recieved a matrix of values
  /*if (memData[dataIndex] == 99) {
    //Print an error if we recieved the wrong length of data
    if (dataIndex != 17 || memData[0] != 0) {
      Serial.println("map matrix formatted incorrectly");
    }
    else {
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          mapDat[i][j] = memData[4 * i + j + 1];
        }
      }
      printMap();
      Serial.println("\t\tMap Recieved");
    }
    memset(memData, 0, 24);
    dataIndex = 0;
    }*/




}//end of loop

void readSerial() {
  if (Serial.available() > 0) {
    data[0] = Serial.parseInt();
    Serial.println(data[0]);
  }
}

void printMap() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (j == 0) {
        Serial.print("\t");
      }
      Serial.print(mapDat[i][j]);
      if (j < 3) {
        Serial.print("\t");
      }
    }
    Serial.print("\n");
  }
}
