/* 
PlatformIO env params:
  [env:teensy]
  platform = teensy
  board = teensy41
  framework = arduino
  monitor_speed = 115200

Teensy USB Communication Example: receive multiple floats over Serial

WARNING: for debugging, DO NOT use Serial.print() or Serial.println()
         as it will interfere with the Serial traffic.

Notes: 
  1) Ensure that the baud rate is set to 115200.
  2) Make sure send and receive message sizes should be agreed upon
     before running the python and arduino code. i.e,. check host computer and this script number of floats
  3) Make sure send/receive frequency [Hz] is also agreed upon.
  3) Run the accompanying python code to receive data.

By Sergio Esteban (sesteban@caltech.edu)
*/

////////////////////////////////////// LIBRARIES //////////////////////////////////////

// Include the necessary libraries
#include <Arduino.h>

////////////////////////////////////// VARIABLES //////////////////////////////////////

// Variables for receiving floats
const int num_float_recv = 7;
float receivedData[num_float_recv];
float x, y, z,        // position  in world frame
      qw, qx, qy, qz, // quaternion in world frame

// TIMING variables
double t0;               // to measure time using internal Teensy clock
const float hz = 150.0;  // desired frequency of data transfer
const float T = 1.0/hz;  // period of data transfer

//////////////////////////////////////// SETUP ////////////////////////////////////////

void setup() {
  
  // Start the serial communication
  Serial.begin(115200); 

  // use builtin LED to show data transmission
  pinMode(LED_BUILTIN, OUTPUT);

  // small delay before beginning
  delay(10);
}

//////////////////////////////////////// LOOP ////////////////////////////////////////

void loop() {
  
  if (micros() - t0 >= (T * 1E6)) {

    digitalWrite(LED_BUILTIN, HIGH);

    t0 = micros();

    // Receive multiple floats over USB
    if (Serial.available() >= sizeof(receivedData)) {
      Serial.readBytes(reinterpret_cast<uint8_t*>(receivedData), sizeof(receivedData));
    }
    x = receivedData[0];
    y = receivedData[1];
    z = receivedData[2];
    qw = receivedData[3];
    qx = receivedData[4];
    qy = receivedData[5];
    qz = receivedData[6];
   
    digitalWrite(LED_BUILTIN, LOW);
  }
}
