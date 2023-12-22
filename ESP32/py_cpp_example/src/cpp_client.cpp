// example client side C/C++ program to demonstrate Socket programming with python server
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

//  this can be anything?
#define PORT 8080

int main(int argc, char const* argv[])
{
	// socket file descriptor, an integer (like a file handle)
	int client_fd;

	int status, valread;
	struct sockaddr_in serv_addr;
	char buffer[1024] = { 0 };
	char* hello = "Hello from client";

    // create client socket with TCP (SOCK_STREAM)
	if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Socket creation error \n");
		return -1;
	}
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
    std::cout << "created socket" << std::endl;

    // set socket options
	if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
    std::cout << "set inet options" << std::endl;
	
	// attempt to connect to server
	if ((status
		= connect(client_fd, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		printf("Connection to Failed\n ");
		return -1;
	}
    std::cout << "connected to server" << std::endl;

    // send hello message to server
	send(client_fd, hello, strlen(hello), 0);
	printf("Hello message sent\n");
	// valread = read(client_fd, buffer, 1024);
	read(client_fd, buffer, 1024);
	printf("from server: %s\n", buffer);

	// close the connected socket
	close(client_fd);
	return 0;
}

