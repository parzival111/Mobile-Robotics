/*
  HEADER

*/

#include <SPI.h>//include serial peripheral interface library
#include <RF24.h>//include wireless transceiver library
#include <nRF24L01.h>//include wireless transceiver library

// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define baud_rate 9600

// IR Definitions
#define irB 0       //back IR
#define irL 1       //left IR
#define irF 2       //front IR
#define irR 3       //right IR
#define cutoff 18   //cutoff for sensors

//variables
boolean transmit = true;              //Start off sending data
boolean readData = true;              //read data from serial
RF24 radio(CE_PIN, CSN_PIN);          //Create instance of radio object
#define team_channel 123              //Set communication channel

const uint64_t pipe = 0xE8E8F0F0E1LL;   //define the radio transmit pipe
uint8_t data[32];                       //variable to hold data
int dataIndex = 0;                      //variable for the index of the last data
uint8_t mapDat[4][4];                   //variable to hold receive data MAP
uint8_t path[8];                        //variable to hold path
uint8_t pos[2];                         //variable to hold start positon
uint8_t goal[2];                        //variable to hold goal

void setup() {
  Serial.begin(baud_rate);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe
  } else {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
  }
}

void loop() {
  if (readData) {
    readSerial();
  }
  else if (transmit) {
    radio.write(data, sizeof(data));
    transmit = false;
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
  }//end of transmit if
  else if (!transmit) {
    while (radio.available()) {
      radio.read(&data, sizeof(data));
      printData();
    }
  }
}

void printData() {
  for (int i = 0; i < sizeof(data); i++) {
    Serial.println(data[i]);
  }
}


void readSerial() {
  if (Serial.available() > 0) {
    data[dataIndex] = Serial.parseInt();
    if (data[dataIndex] == 100) {
      readData = false;
      data[dataIndex] = 0;
    }
    if (data[dataIndex] > 0) {
      dataIndex++;
    }
  }
}
