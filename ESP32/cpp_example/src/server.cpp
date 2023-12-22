// https://www.geeksforgeeks.org/socket-programming-cc/
// https://www.youtube.com/watch?v=eVYsIolL2gE  (god-level explanation)

// Server side C/C++ program to demonstrate Socket
// programming
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

//  this can be anything?
#define PORT 8080

int main(int argc, char const* argv[]){
	
	// socket file descriptor, an integer (like a file handle)
	int server_fd;
	
	int new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = { 0 };
	char* hello = "Hello from server";

// ---------------------------------------------------------------------------------------

	// MAIN PART 1 (SOCKET CREATION): socket creation 
	// Creating socket file descriptor 
	// int sockfd = socket(domain, type, protocol);
	// domain: use AF_INET for IPV4 | use AF_INET6 for IPV6
	// type: TCP use SOCK_STREAM | UDP use SOCK_DGRAM
	// protocol: usually 0
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
	std::cout << "created socket" << std::endl;

// ---------------------------------------------------------------------------------------

	// MAIN PART 2 (SOCKET OPTIONS): manipulating options for the socket referred by the file descriptor (server_fd)
	// helps in reuse of address and port. Prevents error such as: “address already in use”.
	// int setsockopt(int sockfd, int level, int optname,  const void *optval, socklen_t optlen);
	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	std::cout << "set socket options" << std::endl;

// ---------------------------------------------------------------------------------------	

	// MAIN PART 3 (BIND): bind function binds the socket to the address and port number specified in addr(custom data structure).
	// we bind the server to the localhost, hence we use INADDR_ANY to specify the IP address.
	// int listen(int sockfd, int backlog);

	// setting sockaddr_in struct attributes
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr*)&address, sizeof(address))< 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	std::cout << "binded to port " << std::to_string(PORT) << std::endl;

//---------------------------------------------------------------------------------------

	// MAIN PART 4 (LISTEN): puts the server socket in a passive mode, where it waits for the client to approach the server to make a connection.
	// int listen(int sockfd, int backlog);
	// pass in socket file descriptor and backlog which defines the maximum length to which the queue of pending connections for sockfd may grow
	if (listen(server_fd, 3) < 0) {
		perror("listen");
		exit(EXIT_FAILURE);
	}
	std::cout << "listening " << std::to_string(PORT) << " hanging and waiting..." << std::endl;

//---------------------------------------------------------------------------------------

	// MAIN PART 5 (ACCEPT):
	// int new_socket= accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
	// Extracts the first connection request on the queue of pending connections for the listening socket, sockfd, 
	// creates a new connected socket, and returns a new file descriptor referring to that socket. 
	// At this point, the connection is established between client and server, and they are ready to transfer data
	if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
		perror("accept");
		exit(EXIT_FAILURE);
	}
	std::cout << "accepted " << std::to_string(new_socket) << std::endl;

//---------------------------------------------------------------------------------------

	// MAIN PART 6 (READ and WRITE):
	// read (char* s, streamsize n);

	// valread = read(new_socket, buffer, 1024);
	read(new_socket, buffer, 1024);
	std::cout << "received message: " << buffer << std::endl;
	// printf("%s\n", buffer);
	send(new_socket, hello, strlen(hello), 0);
	std::cout << "sent message: " << hello << std::endl;
	// printf("Hello message sent\n");

//---------------------------------------------------------------------------------------

	// MAIN PART 7 (CLOSE):
	// closing the connected socket
	close(new_socket);
	// closing the listening socket
	shutdown(server_fd, SHUT_RDWR);
	std::cout << "socket closed" << std::endl;
	return 0;
}

