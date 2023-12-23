# example of a python server that communicates with a C++ client
import socket

def server_program():
    
    host = '' # get the hostname, in my case, the machine name (if on same PC)
    port = 8080  # initiate port no above 1024, can be anything

    # SOCK_STREAM is TCP and SOCK_DGRAM is UDP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create the server endpoint for commnuication
    print("Server socket created")
    
    # setup server socket options
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # bind host address and port together which makes conn unique
    server_socket.bind((host, port))  
    print("Server socket binded: Host [{}], Port: [{}] ".format(host, port))

    server_socket.listen(1) # assign 1 client to be connected at a time, and wait for client connection
    conn, address = server_socket.accept()  # accept new connection when client connects
    print("Connection from: " + str(address))
   
    num = 12345
    try: 
        while True:
            
            # create message to send to client
            message2client = str(num)  # take input
            conn.send(message2client.encode())  # send data to the client

            # num = num + 1
    except KeyboardInterrupt:
        print("\n\nKeyboard Interrupt. Closing Socket")
        conn.close()  # close the connection
    
    print("Server socket closed.")

if __name__ == '__main__':
    server_program()

