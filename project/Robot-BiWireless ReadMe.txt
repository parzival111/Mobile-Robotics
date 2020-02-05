“Robot-BiWireless.ino“ Read Me File

If you are doing topological or metric path planning and want to send the goal point or topological path to the robot use “1,2,3,4” or “1,1” and press enter or send in the serial monitor. 1,2,3,4 would represent straight, left, right, terminate for example. 1,1 would represent the robot’s goal is the 1,1 cell in the map.

I have filtered out the zero to send or receive since it sends a default zero whenever you press enter.  If you want to send the map or localization coordinates from the robot to your computer, you can also use the code for that by sending “1,1” or “0,0,99,0,0,99,…”[enter 16 data points for a 4 x 4 matrix].  “1,1” is the cell location where the robot has localized, you can also use a single number system from 0 to 15 if you prefer.  For the second set you can send 16 numbers in an occupancy grid or topological map that represent the robot’s map representation.
  
There are some variables that need to be set at the beginning of the code dependent upon what you want to do.
boolean transmit=	//TRUE-sending data, FALSE-receiving data
boolean uno=	//TRUE = laptop sending, FALSE=robot sending

HARDWARE CONNECTIONS:
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
