/*
    Serial node class to communicate with a Teensy 4.1
    over a USB serial connection. 

    0. Makre sure that the packet sizes are agreed upon between the Teensy and the host PC.
    1. After plugging USB cable to your laptop and to the Teensy, looks for the port name
       in the Arduino IDE.
    2. Set buad rate to 115200 in the Arduino IDE.
    3. Run theaccopanying Arduino code. 

    Notes:
        - The Teensy must be connected to the computer via USB cable. 
        - You can still have successful connection when the USB cable is plugged in frpm the laptop
          side but not from the Teensy side.
        - WARNING: You may run into issues if you try to observe UART traffic with Serial Monitor.

    by: Sergio Esteban (sesteban@caltech.edu)
*/

#include "../inc/SerialNode.h"

// Constructor
SerialNode::SerialNode(const std::string &ttyPort_, int baudRate_, int numFloatSend_, int numFloatRecv_) {
    
    // set the number of floats to send and receive as well as vector sizes
    this->numFloatSend = numFloatSend_;
    this->numFloatRecv = numFloatRecv_;
    // this->sendData.resize(this->numFloatSend, 0.0f); // resize and initialize with zeros
    // this->recvData.resize(this->numFloatRecv, 0.0f); // resize and initialize with zeros

    // set the port and baud rate
    this->ttyPort = ttyPort_;
    this->baudRate = baudRate_;

    // print the port and baud rate information
    std::cout << "Opening serial port " << this->ttyPort << " at " << this->baudRate << " baud rate..." << std::endl;

    // open file descriptor at ttyPort where the Teensy is connected
    this->tty_fd = open(this->ttyPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    // check if failed to open the port
    if (this->tty_fd == -1) {
        std::string error = "Error opening serial port " + this->ttyPort + 
                            "\nMake sure that the port name is correct and that the USB cable is connected.";
        perror(error.c_str());
        exit(EXIT_FAILURE);
    }
    else {
        std::cout << "Successfully opened serial port " << this->ttyPort << "." << std::endl;
    }

    // configure the port
    struct termios options;                 // decalre a termios struct
    tcgetattr(this->tty_fd, &options);      // get the current options
    cfsetispeed(&options, this->baudRate);  // set input baud rate
    cfsetospeed(&options, this->baudRate);  // set output baud rate
    options.c_cflag |= (CLOCAL | CREAD); 
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(this->tty_fd, TCSANOW, &options);
}

// Destructor
SerialNode::~SerialNode() {
    // close the file descriptor
    std::cout << "Closing serial port " << this->ttyPort << "..." << std::endl;
    close(this->tty_fd);
    std::cout << "Successfully closed serial port " << this->ttyPort << "." << std::endl;
}

// Write data to the Teensy
void SerialNode::writeData(std::vector<float> data) {

    // check if the data size is correct
    if (data.size() != this->numFloatSend) {
        std::string error = "Error: The size of the data vector must be " + std::to_string(this->numFloatSend) + ".";
        perror(error.c_str());
        exit(EXIT_FAILURE);
    }

    // convert std vector to float array
    float sendData[this->numFloatSend];
    for (int i = 0; i < this->numFloatSend; i++) {
        sendData[i] = data[i];
    }

    // Send multiple floats over USB
    write(this->tty_fd, reinterpret_cast<uint8_t*>(sendData), sizeof(sendData));
}

// Read data from the Teensy
std::vector<float> SerialNode::readData() {

    // container to hold the received data
    float recvData[this->numFloatRecv];
    std::vector<float> data(this->numFloatRecv);   

    // Receive multiple floats over USB if there is anything in the serial buffer
    if (read(this->tty_fd, reinterpret_cast<uint8_t*>(recvData), sizeof(recvData)) >= sizeof(recvData)) {
        for (int i = 0; i < this->numFloatRecv; i++) {
            data[i] = recvData[i];
        }
    }

    return data;
}




