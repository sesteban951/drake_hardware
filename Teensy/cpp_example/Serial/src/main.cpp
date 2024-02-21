// /* 
//   PlatformIO env params:
//     [env:teensy]
//     platform = teensy
//     board = teensy41
//     framework = arduino
//     monitor_speed = 115200

//   Teensy USB Communication Example: send and receive multiple floats over Serial

//   WARNING: for debugging, DO NOT use Serial.print() or Serial.println()
//           as it will interfere with the Serial traffic.

//   Notes: 
//     1) Ensure that the baud rate is set to 115200
//     2) Make sure send and receive message sizes are agreed upon.
//       before running the python and arduino code.
//     3) Make sure send/receive frequency is also agreed upon.
//     3) Run the accompanying python code to receive and send data

//   By Sergio Esteban (sesteban@caltech.edu)
// */

// ////////////////////////////////////// LIBRARIES //////////////////////////////////////

// // Include the necessary libraries
// #include <Arduino.h>

// ////////////////////////////////////// VARIABLES //////////////////////////////////////

// // Variables for sending floats
// const int num_float_send = 6;
// float sendData[num_float_send];
// float q1_send, q2_send, q3_send, // right leg positions 
//       q4_send, q5_send, q6_send; // left leg positions

// // Variables for receiving floats
// const int num_float_recv = 15;
// float receivedData[num_float_recv];
// float px, py, pz,     // position  in world frame
//       qw, qx, qy, qz, // quaternion in world frame
//       escR, escL,     // right and left thruster values
//       q1_recv, q2_recv, q3_recv, // right leg positions
//       q4_recv, q5_recv, q6_recv; // left leg positions

// // TIMING variables
// double time1, time2;           // to measure time using internal ESP32 clock
// double t_elapsed, t_remaining; // to measure time remaining to achieve desired frequency
// const float hz = 300.0;        // desired frequency of data transfer
// const float T = 1.0/hz;        // period of data transfer

// ////////////////////////////////////// FUNCTIONS //////////////////////////////////////

// // arbitrary function to generate dummy data
// float sine_wave(float phi){
//   float t, f, x;
//   t = millis()/1000.0;
//   f = 1.0;
//   x = 500.0 * sin(2 * M_PI * f*  t + phi);
//   return x;
// }

// //////////////////////////////////////// SETUP ////////////////////////////////////////

// void setup() {
  
//   // Start the serial communication
//   Serial.begin(115200); 

//   // use builtin LED to show data transmission
//   pinMode(LED_BUILTIN, OUTPUT);

//   // small delay before beginning
//   delay(10);
// }

// //////////////////////////////////////// LOOP ////////////////////////////////////////

// void loop() {
  
//   digitalWrite(LED_BUILTIN, HIGH);

//   // Send multiple floats over USB
//   q1_send = px;  // TODO: replace with actual data. Right now, I'm just sneding back pose data
//   q2_send = py;
//   q3_send = pz;
//   q4_send = qw;
//   q5_send = qx;
//   q6_send = qy;
//   sendData[0] = q1_send;
//   sendData[1] = q2_send;
//   sendData[2] = q3_send;
//   sendData[3] = q4_send;
//   sendData[4] = q5_send;
//   sendData[5] = q6_send;
//   Serial.write(reinterpret_cast<uint8_t*>(sendData), sizeof(sendData));
  
//   delayMicroseconds(100); // Small delay to not corrupt data

//   // Receive multiple floats over USB
//   if (Serial.available() >= sizeof(receivedData)) {
//     Serial.readBytes(reinterpret_cast<uint8_t*>(receivedData), sizeof(receivedData));
//   }
//   px = receivedData[0];
//   py = receivedData[1];
//   pz = receivedData[2];
//   qw = receivedData[3];
//   qx = receivedData[4];
//   qy = receivedData[5];
//   qz = receivedData[6];
//   escR = receivedData[7];
//   escL = receivedData[8];
//   q1_recv = receivedData[9];
//   q2_recv = receivedData[10];
//   q3_recv = receivedData[11];
//   q4_recv = receivedData[12];
//   q5_recv = receivedData[13];
//   q6_recv = receivedData[14];

//   digitalWrite(LED_BUILTIN, LOW);

//   delayMicroseconds(100); // Small delay to not corrupt data

// }

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

#include <Wire.h>
#define DEVICE_ID 0x33             //I2C Address, Don't change

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

int jj = 0;
unsigned long t_i2c = 0;
unsigned long t_loop = 0;
int dt = 0;;

////////////////////////////////////// FUNCTIONS //////////////////////////////////////

// arbitrary function to generate dummy data
float sine_wave(float phi){
  float t, f, x;
  t = millis()/1000.0;
  f = 1.0;
  x = 500.0 * sin(2 * M_PI * f*  t + phi);
  return x;
}

int saturateInt(int x, int xMin, int xMax)
{
  if (x > xMax)
    return xMax;
  else if (x < xMin)
    return xMin;
  else
    return x;
}

String convertValue2String(float value)
{
  String output;

  // 255^2 = 65025, maybe use max 65024 instead, 65024/2 = 32512
  // 2 bytes signed integer, apply saturation

  //  int value_uint8 = floor(value * 1000) + 32768;
  //  int x = saturateInt(value_uint8, 0, 65535);
  //  output += (char)(x / 256);
  //  output += (char)(x % 256);

  int value_uint8 = floor(value * 1000) + 32512;
  int x = saturateInt(value_uint8, 0, 65024);
  output += (char)((x / 255) + 1);
  output += (char)((x % 255) + 1);

  return output;
}

float bytes2float(char b1, char b2) {
  float output = 0;
  int value = b1 * 256 + b2; // uint8_t
  output = (float)(value - 32768) / 1000.0;
  return output;
}

void requestEvent()
{
  t_i2c = micros();
  // DEBUG
  jj++;
  if (jj > 100) {
    jj = 0;
  }

  float x  = jj * 0.6 - 30;
  float y  = jj * 0.3 - 20;
  float z  = jj * 0.3 - 10;
  float q1 = jj * 0.001;
  float q2 = jj * 0.001 + 5;
  float q3 = jj * 0.002 + 10;
  float q4 = jj * 0.003 + 15;
  
  // send data through i2c on master request
  String data;
  data += convertValue2String(px);
  data += convertValue2String(py);
  data += convertValue2String(pz);

  // data += convertValue2String(x);
  // data += convertValue2String(y);
  // data += convertValue2String(z);

  // data += convertValue2String(qx);
  // data += convertValue2String(qy);
  // data += convertValue2String(qz);
  // data += convertValue2String(qw);

  data += convertValue2String(x);
  data += convertValue2String(qy);
  data += convertValue2String(qz);
  data += convertValue2String(qw);
  
  data += (char) 0; // stop byte

  Wire.write(data.c_str());

  // Serial.print("T: ");
  // Serial.print(micros() - t_i2c);
  // Serial.println();
}

void receiveEvent(int bytes) {
  t_i2c = micros();

  // receive data on master send
  String i2cData;
  while (Wire.available()) {
    char c = Wire.read();
    i2cData += c;
  }

  // Parse data here
  if (i2cData.length() == 20) 
  {
    // q1_send = bytes2float(i2cData[0], i2cData[1]);
    // q2_send = bytes2float(i2cData[2], i2cData[3]);
    // q3_send = bytes2float(i2cData[4], i2cData[5]);
    // q4_send = bytes2float(i2cData[6], i2cData[7]);
    // q5_send = bytes2float(i2cData[8], i2cData[9]);
    // q6_send = bytes2float(i2cData[10], i2cData[11]);

    // thrusters = bytes2float(i2cData[0], i2cData[1]);
    // thrusters = bytes2float(i2cData[2], i2cData[3]);

    q1_send = bytes2float(i2cData[4], i2cData[5]);
    q2_send = bytes2float(i2cData[6], i2cData[7]);
    q3_send = bytes2float(i2cData[8], i2cData[9]);
    q4_send = bytes2float(i2cData[10], i2cData[11]);
    q5_send = bytes2float(i2cData[12], i2cData[13]);
    q6_send = bytes2float(i2cData[14], i2cData[15]);
  }
  q6_send = 12.3; // DEBUG

  // DEBUG
  // Serial.print("Length: ");
  // Serial.print(i2cData.length());
  // Serial.print(", msg: ");
  // Serial.println(i2cData);

  // Serial.print("R: ");
  // Serial.print(micros() - t_i2c);
  // Serial.println();
}

//////////////////////////////////////// SETUP ////////////////////////////////////////

void setup() {
  
  // Start the serial communication
  Serial.begin(115200); 

  // use builtin LED to show data transmission
  pinMode(LED_BUILTIN, OUTPUT);
  
  // i2c
  Wire.begin(DEVICE_ID);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // small delay before beginning the loop
  delay(1000);
}

//////////////////////////////////////// LOOP ////////////////////////////////////////

void loop() {

if (micros() - t_loop >= 10000){
  dt = micros() - t_loop;
  t_loop = micros();
  


  digitalWrite(LED_BUILTIN, HIGH);

  // Send multiple floats over USB
  // q1_send = px;  // TODO: replace with actual data. Right now, I'm just sneding back pose data
  // q2_send = py;
  // q3_send = pz;
  // q4_send = qw;
  // q5_send = qx;
  // q6_send = qy;
  sendData[0] = q1_send;
  sendData[1] = q2_send;
  sendData[2] = q3_send;
  sendData[3] = q4_send;
  sendData[4] = q5_send;
  sendData[5] = q6_send;
  
  Serial.write(reinterpret_cast<uint8_t*>(sendData), sizeof(sendData)); //DEBUG
  
  delayMicroseconds(100); // Small delay to not corrupt data

  // Receive multiple floats over USB
  if (Serial.available() >= sizeof(receivedData)) {
    Serial.readBytes(reinterpret_cast<uint8_t*>(receivedData), sizeof(receivedData)); //DEBUG
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

  // Serial.println(px);

  digitalWrite(LED_BUILTIN, LOW);

  // delayMicroseconds(100); // Small delay to not corrupt data
  //Serial.println(dt);

  // Serial.print("q: "); Serial.print(q1_send);
  // Serial.print(", ");  Serial.print(q2_send);
  // Serial.print(", ");  Serial.print(q3_send);
  // Serial.print(", ");  Serial.print(q4_send);
  // Serial.print(", ");  Serial.print(q5_send);
  // Serial.print(", ");  Serial.print(q6_send);
  // Serial.println();

}

}
