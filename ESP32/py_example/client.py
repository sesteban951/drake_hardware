import socket

def client_program():
    host = socket.gethostname()  # get the hostname, in my case, the machine name (if on same PC)
    port = 8080  # socket server port number, need to have same port as server

    # SOCK_STREM is TCP and SOCK_DGRAM is UDP
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create the client endpoint for commnuication
    client_socket.connect((host, port))  # attempt to connect to server at host and port

    message2server = None
    while message2server != 'q':

        # create message to send to server
        message2server = input("to server: ")  # take input
        client_socket.send(message2server.encode())  # send message to client

        # receive data stream. it won't accept data packet greater than 1024 bytes
        data = client_socket.recv(1024).decode() 
        print('from server: ' + str(data))  # show in terminal

    client_socket.close()  # close the connection

if __name__ == '__main__':
    client_program()

