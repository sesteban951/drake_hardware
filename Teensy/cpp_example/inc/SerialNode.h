/*
    Serial node class to communicate with a Teensy 4.1
    over a USB serial connection. 

    0. Make sure that the packet sizes are agreed upon between the Teensy and the host PC.
    1. After plugging USB cable to your laptop and to the Teensy, look for the port name
       in the Arduino IDE.
    2. Set buad rate to 115200 in the Arduino IDE.
    3. Run the accompanying Arduino code. 

    Notes:
        - The Teensy must be connected to the computer via USB cable. 
        - You can still have successful connection when the USB cable is plugged in from the laptop
          side but not from the Teensy side.
        - WARNING: You may run into issues if you try to observe UART traffic with Serial Monitor.

    by: Sergio Esteban (sesteban@caltech.edu)
*/

// include this once per compialtion
#pragma once

// C++ Standard Library
#include <iostream> 
#include <unistd.h>  
#include <fcntl.h>   
#include <termios.h> 
#include <chrono>
#include <thread>
#include <vector>

class SerialNode{

    public:

        // Constructor and Destructor
        SerialNode(const std::string &ttyPort_, int baudRate_, int numFloatSend_, int numFloatRecv_);
        ~SerialNode();

        // read and write functions
        void writeData(std::vector<float> data);
        std::vector<float> readData();

    private:
        
        // port, baud rate, and tty file descriptor
        std::string ttyPort;
        int baudRate;
        int tty_fd;

        // send and receive data containers
        int numFloatSend;  // number of floats to send
        int numFloatRecv;  // number of floats to receive

};