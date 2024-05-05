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
           use "ifconfig" to get wlp inet addr on server from laptop (must do this)

  Zero, ensure that byte sizes are agreed upon from server and client (i.e, adjust container sizes below)
  First, run accopanying py_server.py script on server (laptop) to receive data from ESP32
  Second, run this script on ESP32 to send and receive from server (laptop)

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
const char* pc_inet = "10.1.1.4"; // use "ifconfig" to get wlp inet addr on server from laptop

// SOCKET server information
#define PORT 8080
int client_fd;   // socket file descriptor, an integer (like a file handle)
int status;      // for connection error checking
struct sockaddr_in serv_addr;

// Receiving message containers
const int send_num_floats = 4;                    // change size as needed, each float is 4 bytes
char recvBuffer[sizeof(float) * send_num_floats]; // buffer for receviing data
ssize_t bytes_read;                               // for meausring received packet size
float recv_num1, recv_num2, recv_num3, recv_num4; // containers for unpacked floats

// Sending message containers
const int recv_num_floats = 4;                    // change size as needed, each float is 4 bytes
float dataToSend[recv_num_floats];                // change size as needed
char sendBuffer[sizeof(float) * recv_num_floats]; // buffer for sending data
float send_num1, send_num2, send_num3, send_num4; // containers for to pack flaots

// TIMING variables
double time1, time2;           // to measure time using internal ESP32 clock
double t_elapsed, t_remaining; // to measure time remaining to achieve desired frequency
const int hz = 100;             // desired frequency of data transfer
const float T = 1.0/hz;        // period of data transfer

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

// arbitrary to generate data
float sine_wave(){
  double t, f, x;
  t = millis()/1000.0;
  f = 1.0;
  x = 500.0 * sin(2 * M_PI * f*  t);
  return x;
}

////////////////////////////////////// SETUP //////////////////////////////////////

void setup(){

  // setup serial
  Serial.begin(115200);
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
  
}

////////////////////////////////////// LOOP //////////////////////////////////////

void loop(){
    
    time1 = micros(); // start timer
    
    // Read the data into the buffer
    bytes_read = read(client_fd, recvBuffer, sizeof(recvBuffer));

    // Unpack the received byte stream into floats
    memcpy(&recv_num1, &recvBuffer[0 * sizeof(float)], sizeof(float));
    memcpy(&recv_num2, &recvBuffer[1 * sizeof(float)], sizeof(float));
    memcpy(&recv_num3, &recvBuffer[2 * sizeof(float)], sizeof(float));
    memcpy(&recv_num4, &recvBuffer[3 * sizeof(float)], sizeof(float));

    // Use received data as needed
    Serial.print("Received floats: (");
    Serial.print(recv_num1,5);
    Serial.print(", ");
    Serial.print(recv_num2,5);
    Serial.print(", ");
    Serial.print(recv_num3,5);
    Serial.print(", ");
    Serial.print(recv_num4,5);
    Serial.println(")");

    delay(1); // small delay to not corrupt data

    // Prepare the data to send to the server (floats)
    send_num1 = sine_wave();
    send_num2 = 245.718281828459045;
    send_num3 = 000000.0000000;
    send_num4 = -31654564; 
    dataToSend[0] = send_num1;
    dataToSend[1] = send_num2;
    dataToSend[2] = send_num3;
    dataToSend[3] = send_num4;

    // Copy the float data into the send buffer
    for (int i = 0; i < 4; ++i) {
        memcpy(&sendBuffer[i * sizeof(float)], &dataToSend[i], sizeof(float));
    }

    // Send the data back to the server
    write(client_fd, sendBuffer, sizeof(sendBuffer));

    time2 = micros(); // stop timer

    // measure communcaiton frequency perfromance
    t_elapsed = (time2-time1) * 0.001;   // [us]
    t_remaining = T*1000.0 - t_elapsed;  // [us]

    // delay to achieve desired frequency
    if (t_remaining > 0) {
      delay(t_remaining); 
    }
    // small delay to not corrupt data
    else {
      delay(1);            
    }

    // print frequency
    time2 = micros(); // stop timer
    Serial.print("Freq. [hz]]: ");
    Serial.println(1.0/(time2-time1)*1000000.0);
}



