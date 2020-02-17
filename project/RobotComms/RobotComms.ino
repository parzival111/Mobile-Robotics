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
#define cutoff 18             //cutoff for sensors

//variables
boolean transmit = false;             //Start off recieving data
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
  }
  else {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
  }
}
void loop() {
  if (transmit) {
    ////// Use this code to test sending with the serial port
    createMessage();
    radio.write(data, sizeof(data));
    delay(1000);
  }//end of transmit if

  else if (!transmit) {
    while (radio.available()) {
      radio.read(&data, sizeof(data));
      saveData();
      printData();
      transmit = true;
      radio.stopListening();
      radio.openWritingPipe(pipe);//open up writing pipe
      delay(1500);
    }
  }
}


void saveData() {
  dataIndex = 0;
  // Read the pos
  pos[0] = data[dataIndex];
  dataIndex++;
  pos[1] = data[dataIndex];
  dataIndex++;
  // Read the goal
  goal[0] = data[dataIndex];
  dataIndex++;
  goal[1] = data[dataIndex];
  dataIndex++;
  // Read the map
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mapDat[i][j] = data[dataIndex];
      dataIndex++;
    }
  }
  // Read the path
  int i2 = 0;
  for (int i = dataIndex; i < sizeof(data); i++) {
    if (data[i] == 0 && i2 == 0) {
      i2 = i;
    }
  }
  Serial.println("Path");
  for(int i = dataIndex; i < i2; i++){
    path[i] = data[i];
    Serial.println(path[i]);
  }
}

void printData() {
  Serial.println("Data: ");
  for (int i = 0; i < sizeof(data); i++) {
    Serial.println(data[i]);
  }
}

void createMessage() {
  double irFront = readIRFront();
  double irRight = readIRRight();
  double irBack = readIRBack();
  double irLeft = readIRLeft();

  dataIndex = 0;
  data[dataIndex] = 101;
  dataIndex++;
  data[dataIndex] = pos[0];
  dataIndex++;
  data[dataIndex] = pos[1];
  dataIndex++;
  data[dataIndex] = goal[0];
  dataIndex++;
  data[dataIndex] = goal[1];
  dataIndex++;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      data[dataIndex] = mapDat[i][j];
      dataIndex++;
    }
  }
  data[dataIndex] = irFront;
  dataIndex++;
  data[dataIndex] = irRight;
  dataIndex++;
  data[dataIndex] = irBack;
  dataIndex++;
  data[dataIndex] = irLeft;
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
