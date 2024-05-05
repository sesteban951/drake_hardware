/*
PlatfromIO env params: 
  Simple example as ESP32 client:
  [env:featheresp32]
  platform = espressif32
  board = featheresp32
  framework = arduino
  monitor_speed = 115200

Notes:
  ssid: WIFI router name
  password: WIFI router password
  pc_inet: IP address of the server (laptop) to connect to
           use "ifconfig" (linux) or "ipconfig" (windows) to get wlp inet addr on server from laptop (must do this)

  1) ensure that number of  floats are agreed upon from server and client (i.e, adjust container sizes below)
  2) run accopanying py_server_send_only.py script on server (on the laptop)
  3) upload this script to ESP32, and run to send and receive from server (on the laptop)

By Sergio Esteban (sesteban@caltech.edu)

*/ 

////////////////////////////////////// LIBRARIES //////////////////////////////////////

// standrad arduino libraries
#include <Arduino.h>
#include <WiFi.h>

// standrad C++ libraries for socket programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

////////////////////////////////////// VARIABLES //////////////////////////////////////

// WIFI router information, must be modified when you change router or laptop
const char* ssid = "NETGEAR46";
const char* password = "amberlab";
const char* pc_inet = "192.168.1.6"; // use "ifconfig"/"ipconfig" to get wlp inet addr on server from laptop

// SOCKET server information
#define PORT 8081
int client_fd;   // socket file descriptor, an integer (like a file handle)
int status;      // for connection error checking
struct sockaddr_in serv_addr;
int baud_rate = 115200;

// Receiving message containers, Laptop --> ESP32
const int recv_num_floats = 7;                    // change size as needed, each float is 4 bytes
char recvBuffer[sizeof(float) * recv_num_floats]; // buffer for receviing data
ssize_t bytes_read;                               // for meausring received packet size
float x, y, z,        // position containers
      qw, qx, qy, qz; // quaternion containers

// TIMING variables
double t0, t1;               // to measure time using internal ESP32 clock
const float hz = 150.0;  // desired frequency of data transfer
const float T = 1.0/hz;  // period of data transfer

////////////////////////////////////// FUNCTIONS //////////////////////////////////////

// function to setup the Wifi connection
void setup_wifi(){

  // WIFI_STA: Station mode (connect to wifi router/network)
  // WIFI_AP: Access point mode (host wifi network)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting to Wifi network...");

  // Attempt to connect to the WiFi network
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }
}

// function to print info once the ESP32 is connected to the Wifi network
void show_network_info(){

  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nSuccessfully to the WiFi network");

    Serial.print("[*] Network information for ");
    Serial.println(ssid);

    Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
    Serial.print("[+] Gateway IP : ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[+] Subnet Mask : ");
    Serial.println(WiFi.subnetMask());
    Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
    Serial.print("[+] ESP32 IP : ");
    Serial.println(WiFi.localIP());

    Serial.println("*****************************************");
  }
}

// function to setup a client socket to connect to server over Wifi
int setup_socket(){
  
  // create client socket with TCP (SOCK_STREAM), if you want UDP (SOCK_DGRAM)
	if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		Serial.print("\n Socket creation error \n");
		return -1;
	}
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
  Serial.println("Created socket.");

  // set socket options
	if (inet_pton(AF_INET, pc_inet, &serv_addr.sin_addr) <= 0) {
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
  Serial.println("Set inet options.");  

  // attempt to connect to server
  Serial.println("Attemptng to Connect to server..."); 
	if ((status
		= connect(client_fd, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		Serial.print("Connection to Server Failed!\n");
		return -1;
	}
  Serial.println("Connected to Server Successfully!");

  return 0;
}

////////////////////////////////////// SETUP //////////////////////////////////////

void setup(){

  // setup serial
  Serial.begin(baud_rate);
  delay(100);

  // enable LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);

  // setup wifi, connect to wifi router
  setup_wifi();
  delay(100);

  // display all network info
  show_network_info();
  delay(100);
  
  // setup client socket
  int socket_status = setup_socket();
  delay(100);

  // if socket setup failed, blink LED
  if (socket_status == -1) {
    Serial.println("Error setting up socket.");
    while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  // else, turn on solid LED to indicate successful socket setup
  else {
    Serial.println("Socket setup successful.");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  
  t0 = micros();
}

////////////////////////////////////// LOOP //////////////////////////////////////

void loop(){
    
    if (micros() - t0 >= (T * 1E6)) {

      digitalWrite(LED_BUILTIN, HIGH);
    
      t0 = micros();
      
      // Read the data into the buffer
      bytes_read = read(client_fd, recvBuffer, sizeof(recvBuffer));

      // Unpack the received byte stream into floats
      memcpy(&x,    &recvBuffer[0 * sizeof(float)], sizeof(float));
      memcpy(&y,    &recvBuffer[1 * sizeof(float)], sizeof(float));
      memcpy(&z,    &recvBuffer[2 * sizeof(float)], sizeof(float));
      memcpy(&qw,   &recvBuffer[3 * sizeof(float)], sizeof(float));
      memcpy(&qx,   &recvBuffer[4 * sizeof(float)], sizeof(float));
      memcpy(&qy,   &recvBuffer[5 * sizeof(float)], sizeof(float));
      memcpy(&qz,   &recvBuffer[6 * sizeof(float)], sizeof(float));

      // Use received data as needed
      Serial.print("Received floats: (");
      Serial.print(x ,4);
      Serial.print(", ");
      Serial.print(y, 4);
      Serial.print(", ");
      Serial.print(z, 4);
      Serial.print(", ");
      Serial.print(qw, 4);
      Serial.print(", ");
      Serial.print(qx, 4);
      Serial.print(", ");
      Serial.print(qy, 4);
      Serial.print(", ");
      Serial.print(qz, 4);
      Serial.println(")");

      digitalWrite(LED_BUILTIN, LOW);
    }
}



