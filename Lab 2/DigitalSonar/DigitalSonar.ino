/*DigitalSonar.ino
  4 pin HRC - SR04 sesnro
  trigger and echo pin tied together on pin 12
  data pin 7
  VCC 5V
  GND GND
*/

#include <NewPing.h>
const int PIN_L = A8; 
const int PIN_R = A9; 
NewPing sonarL(PIN_L, PIN_L, 200);
NewPing sonarR(PIN_R, PIN_R, 200);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay
  unsigned int uL = sonarL.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int uR = sonarR.ping(); // Send ping, get ping time in microseconds (uS).
  
  Serial.print(uL);
  Serial.print(" | ");
  Serial.println(uR);
  
  
}
