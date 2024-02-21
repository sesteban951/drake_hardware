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
#include <cmath>

// to get the current time in milliseconds
int64_t get_time_millis() {
    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    return milliseconds.count();
}

// sine wave generator
float sineWaveGenerator(float t) {
    double A = 1.0;
    double f = 0.5;
    return A * std::sin(2 * 3.14159 * f * t);
}

// main communication loop
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

    // for controlling the loop rate
    int64_t t0 = 0; 
    int64_t t1 = 0;
    int64_t T = 10; // 10  [ms]
    float s_val;
    double period, frequency;
    long k = 0;

    // small delay before beginning the communication
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

    // main loop write and read
    while (true) {

        t1 = get_time_millis();

        // if period has passed, then write and read data
        if (t1 - t0 >= T) {

            k++;
            float t_sine = k * 0.01;
            s_val = sineWaveGenerator(t_sine);
            std::cout << "sine wave: " << s_val << std::endl;

            // Print the frequency in Hz
            double period = (t1 - t0) / 1000.0;
            double frequency = 1.0 / period;
            std::cout << "Hz: " << frequency << std::endl;
         
            t0 = get_time_millis();

            // write data to the Teensy
            // sendData[0] = s_val;
            sendData[0] = 0.5* s_val;
            sendData[1] = 1.0 * s_val;
            sendData[2] = 2.0* s_val;
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

        }
    }
    
    return 0;
}
