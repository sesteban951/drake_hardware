#include <iostream>  // printing

#include <unistd.h>  // read, write, close
#include <fcntl.h>   // usb I/O
#include <termios.h> // usb I/O

#include <chrono>
#include <thread>

////////////////////////////////////// VARIABLES //////////////////////////////////////

// Variables for receiving floats
const int numFloatSend = 15;
float sendData[numFloatSend];
float px, py, pz,     // position  in world frame
      qw, qx, qy, qz, // quaternion in world frame
      escR, escL,     // right and left thruster values
      q1_send, q2_send, q3_send, // right leg positions
      q4_send, q5_send, q6_send; // left leg positions

// Variables for sending floats
const int numFloatRecv = 6;
float recvData[numFloatRecv];
float q1_recv, q2_recv, q3_recv, // right leg positions 
      q4_recv, q5_recv, q6_recv; // left leg positions

//////////////////////////////////////// SETUP ////////////////////////////////////////

int setupSerial(const std::string &ttyPort, int baudRate) {

    // print the port and baud rate information
    std::cout << "Opening serial port " << ttyPort << " at " << baudRate << " baud rate..." << std::endl;

    // open file descriptor at ttyPort where the Teensy is connected
    int tty_fd = open(ttyPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    // check if failed to open the port
    if (tty_fd == -1) {
        std::string error = "Error opening serial port " + ttyPort + 
                            "\nMake sure that the port name is correct and that the USB cable is connected.";
        perror(error.c_str());
        exit(EXIT_FAILURE);
    }
    else {
        std::cout << "Successfully opened serial port " << ttyPort << "." << std::endl;
    }

    // configure the port
    struct termios options;              // decalre a termios struct
    tcgetattr(tty_fd, &options);         // get the current options
    cfsetispeed(&options, baudRate);     // set input baud rate
    cfsetospeed(&options, baudRate);     // set output baud rate
    options.c_cflag |= (CLOCAL | CREAD); 
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(tty_fd, TCSANOW, &options);

    return tty_fd;
}

//////////////////////////////////////// LOOP ////////////////////////////////////////

int main() {
    
    // define port and baud rate
    std::string ttyPort = "/dev/ttyACM0";  // Replace with your actual serial port
    int baudRate = 115200;

    // attempt to open the serial port
    int serial_fd = setupSerial(ttyPort, baudRate);

    // small delay before beginning
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

    int c = 0;
    while (true) {
        c++;
        // Send multiple floats over USB
        sendData[0] = c;
        sendData[1] = 2.0;
        sendData[2] = 3.0;
        sendData[3] = 4.0;
        sendData[4] = 5.0;
        sendData[5] = 6.0;
        sendData[6] = qz;
        sendData[7] = escR;
        sendData[8] = escL;
        sendData[9] = q1_send;
        sendData[10] = q2_send;
        sendData[11] = q3_send;
        sendData[12] = q4_send;
        sendData[13] = q5_send;
        sendData[14] = q6_send;
        write(serial_fd, reinterpret_cast<uint8_t*>(sendData), sizeof(sendData));

        // small delay to not corrupt data
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        // Receive multiple floats over USB
        if (read(serial_fd, reinterpret_cast<uint8_t*>(recvData), sizeof(recvData)) >= sizeof(recvData)) {
            q1_recv = recvData[0];
            q2_recv = recvData[1];
            q3_recv = recvData[2];
            q4_recv = recvData[3];
            q5_recv = recvData[4];
            q6_recv = recvData[5];
        }

        // print the received data
        std::cout << "Received data: " << q1_recv << " " << q2_recv << " " << q3_recv << " " 
                  << q4_recv << " " << q5_recv << " " << q6_recv << std::endl;

        // small delay to not corrupt data
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    close(serial_fd);  // Close the serial port
    return 0;
}

