/*DigitalSonar.ino
  4 pin HRC - SR04 sesnro
  trigger and echo pin tied together on pin 12
  data pin 7
  VCC 5V
  GND GND
*/

#define irF 0
#define irR 1
#define irB 2
#define irL 3


void setup() {
  Serial.begin(9600);
  
}

void loop() {
  int value = 0;
  for (int i = 0; i < 19; i++){
    value = value + analogRead(irR);
  }
  value = value/20;
  Serial.println(value);
  delay(100);
}
