/*
PlatfromIO env params: 
  Simple example as ESP32 client:
  [env:featheresp32]
  platform = espressif32
  board = featheresp32
  framework = arduino
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

// WIFI router information
const char* ssid = "NETGEAR22";
const char* password = "perfectfire175";
const char* pc_inet = "10.1.1.9"; // use "ifconfig" to get wlp inet addr on server from laptop

// SOCKET server information
#define PORT 8080
WiFiClient client;

int client_fd; // socket file descriptor, an integer (like a file handle)
int status, received_msg;
struct sockaddr_in serv_addr;
char buffer[4] = { 0 };

// TIMING variables
double t0 = 0.0;
double tf = 0.0;

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

// setup a client socket to connect to server over Wifi
int setup_socket(){
  
  // create client socket with TCP (SOCK_STREAM)
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
  Serial.println("Connected to Server Success!");

  // Connect to server
  // if (!client.connect(pc_inet, PORT)) {
  //   Serial.println("Connection to server failed");
  //   return -1;
  // } else {
  //   Serial.println("Connected to server");
  //   return 0;
  // }
  return 0;
}

////////////////////////////////////// SETUP //////////////////////////////////////

void setup(){

  // setup serial
  Serial.begin(115200);
  delay(500);

  // enable LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);

  // setup wifi
  setup_wifi();
  delay(100);

  // display all netwrok info
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
      delay(75);
      digitalWrite(LED_BUILTIN, LOW);
      delay(75);
    }
  }
  // else, turn on solid LED
  else {
    Serial.println("Socket setup successful.");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
  }
  
  t0 = millis();
}

////////////////////////////////////// LOOP //////////////////////////////////////

void loop(){
    // print current time
    tf = (millis() - t0)/1000.0;
    Serial.println(tf);
    
    // read message from server
    ssize_t bytes_read = read(client_fd, buffer, 5);
    buffer[bytes_read] = '\0'; // null terminate the string
    String message = String(buffer);
    Serial.println(message); 

    // if (client.available()) {
    //     String message = client.readString(); // read the incoming data as a string
    //     Serial.println(message); // print the received message
    //     Serial.println();
    // } 
    
}



