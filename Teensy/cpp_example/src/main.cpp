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

int main(){
    
    // set the port and baud rate
    std::string ttyPort = "/dev/ttyACM0";
    int baud_rate = 115200;

    // send and receive data settings
    int sendSize = 15;
    int recvSize = 6;

    // create a SerialNode object
    SerialNode s = SerialNode(ttyPort, baud_rate, sendSize, recvSize);
    
    // create send and receive data vector containers
    std::vector<float> sendData(sendSize, 0.0f);
    std::vector<float> recvData(recvSize, 0.0f);

    // fill send vector with dummy data
    for (int i = 0; i < sendSize; i++) {
        sendData[i] = i * 3.14159;
    }

    // small delay before beginning
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

    // main loop write and read
    while (true) {
        // write data to the Teensy
        s.writeData(sendData);

        // small delay to not corrupt data
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        // read data from the Teensy
        recvData = s.readData();

        // print the received data
        std::cout << "Received data: ";
        for (int i = 0; i < recvSize; i++) {
            std::cout << recvData[i] << " ";
        }
        std::cout << std::endl;

        // small delay to not corrupt data
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    return 0;
}