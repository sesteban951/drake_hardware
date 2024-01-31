/* 
PlatformIO env params:
  [env:teensy]
  platform = teensy
  board = teensy41
  framework = arduino
  monitor_speed = 115200

Teensy USB Communication Example: send and receive multiple floats over Serial

WARNING: for debugging, DO NOT use Serial.print() or Serial.println()
         as it will interfere with the Serial traffic.

Notes: 
  1) Ensure that the baud rate is set to 115200
  2) Make sure send and receive message sizes should be agreed upon.
     before running the python and arduino code.
  3) Make sure send/receive frequency is also agreed upon.
  3) Run the accompanying python code to receive and send data

By Sergio Esteban (sesteban@caltech.edu)
*/

////////////////////////////////////// LIBRARIES //////////////////////////////////////

// Include the necessary libraries
#include <Arduino.h>

////////////////////////////////////// VARIABLES //////////////////////////////////////

// Variables for sending floats
const int num_float_send = 6;
float sendData[num_float_send];
float q1_send, q2_send, q3_send, // right leg positions 
      q4_send, q5_send, q6_send; // left leg positions

// Variables for receiving floats
const int num_float_recv = 15;
float receivedData[num_float_recv];
float px, py, pz,     // position  in world frame
      qw, qx, qy, qz, // quaternion in world frame
      escR, escL,     // right and left thruster values
      q1_recv, q2_recv, q3_recv, // right leg positions
      q4_recv, q5_recv, q6_recv; // left leg positions

// TIMING variables
double time1, time2;           // to measure time using internal ESP32 clock
double t_elapsed, t_remaining; // to measure time remaining to achieve desired frequency
const float hz = 300.0;        // desired frequency of data transfer
const float T = 1.0/hz;        // period of data transfer

////////////////////////////////////// FUNCTIONS //////////////////////////////////////

// arbitrary function to generate dummy data
float sine_wave(float phi){
  float t, f, x;
  t = millis()/1000.0;
  f = 1.0;
  x = 500.0 * sin(2 * M_PI * f*  t + phi);
  return x;
}

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
  
  digitalWrite(LED_BUILTIN, HIGH);

  // Send multiple floats over USB
  q1_send = px;  // TODO: replace with actual data. Right now, I'm just sneding back pose data
  q2_send = py;
  q3_send = pz;
  q4_send = qx;
  q5_send = qy;
  q6_send = qz;
  sendData[0] = q1_send;
  sendData[1] = q2_send;
  sendData[2] = q3_send;
  sendData[3] = q4_send;
  sendData[4] = q5_send;
  sendData[5] = q6_send;
  Serial.write(reinterpret_cast<uint8_t*>(sendData), sizeof(sendData));
  
  delay(1); // Small delay to not corrupt data

  // Receive multiple floats over USB
  if (Serial.available() >= sizeof(receivedData)) {
    Serial.readBytes(reinterpret_cast<uint8_t*>(receivedData), sizeof(receivedData));
  }
  px = receivedData[0];
  py = receivedData[1];
  pz = receivedData[2];
  qw = receivedData[3];
  qx = receivedData[4];
  qy = receivedData[5];
  qz = receivedData[6];
  escR = receivedData[7];
  escL = receivedData[8];
  q1_recv = receivedData[9];
  q2_recv = receivedData[10];
  q3_recv = receivedData[11];
  q4_recv = receivedData[12];
  q5_recv = receivedData[13];
  q6_recv = receivedData[14];

  digitalWrite(LED_BUILTIN, LOW);

  delay(1); // Small delay to not corrupt data

}
