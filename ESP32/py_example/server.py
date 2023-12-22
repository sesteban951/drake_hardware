import socket

def server_program():
    
    host = socket.gethostname() # get the hostname, in my case, the machine name (if on same PC)
    port = 8080  # initiate port no above 1024, can be anything

    # SOCK_STREM is TCP and SOCK_DGRAM is UDP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create the server endpoint for commnuication
    server_socket.bind((host, port))  # bind host address and port together which makes conn unique

    server_socket.listen(2) # assign 2 clients to be connected at a time, and wait for client connection
    conn, address = server_socket.accept()  # accept new connection when client connects
    
    print("Connection from: " + str(address))
   
    while True:
        # receive data stream. it won't accept data packet greater than 1024 bytes
        data = conn.recv(1024).decode()
        print("from client: " + str(data))

        # id no data is received, break
        if not data:
            # if data is not received break
            break
        
        # create message to send to client
        message2client = input("to client: ")  # take input
        conn.send(message2client.encode())  # send data to the client

    conn.close()  # close the connection

if __name__ == '__main__':
    server_program()
