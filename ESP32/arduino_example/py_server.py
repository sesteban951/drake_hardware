# example of a python server that communicates with a C++ client

import socket # to use socket prgramming
import struct # to use structs  when sending data
import time   # to watch the communication frequency
import numpy as np

import random

# arbitrary function to generate data
def sine_wave():
    t = time.time()
    f = 1.0
    x = 10.0 * np.sin(2 * np.pi * f * t)
    
    # # clip to get square wave
    # if x > 0:
    #     x = 1
    # else:
    #     x = 0

    return  x

def generate_pos():
    px = random.random()-0.5
    py = random.random()-0.5
    pz = random.random()-0.5
    pos = np.array([px, py, pz])

    return px, py, pz

def generate_quat():
    qw = random.random()-0.5
    qx = random.random()-0.5
    qy = random.random()-0.5
    qz = random.random()-0.5
    quat = np.array([qw,qx,qy,qz])
    quat = quat / np.linalg.norm(quat)
    
    return quat  

# server program to communicate with a client over the same wifi network
def server_program():

    # send and receive variables
    send_num_floats = 4
    send_f_str = 'f' * send_num_floats # number of f's is number of floats
    send_num1 = 3654.1415926
    send_num2 = -245.718281828459045
    send_num3 = 000000.0000000
    send_num4 = 31654564
    send_num5 = 3654.1415926
    send_num6 = -245.718281828459045
    send_num7 = 000000.0000000
    send_num8 = 31654564

    recv_num_floats = 4
    recv_f_str= 'f' * recv_num_floats

    # timing variables
    hz = 100 # desired communication frequency

    # network variables
    host = ''    # get the hostname, in my case, the machine name (if on same PC)
    port = 8080  # initiate port no above 1024, can be anything

    # SOCK_STREAM is TCP and SOCK_DGRAM is UDP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create the server endpoint for commnuication
    print("Server socket created")
    
    # setup server socket options
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # bind host address and port together which makes a unique connection
    server_socket.bind((host, port))  
    print("Server socket binded: Host [{}], Port: [{}] ".format(host, port))
    print("Waiting for connection...")

    server_socket.listen(1)                   # assign 1 client to be connected at a time, and wait for client connection
    conn, address = server_socket.accept()    # accept new connection when client connects
    print("Connection from: " + str(address))

    # loop the send/read over wifi
    try: 
        while True:

            time1 = time.time() # start timer

            # send message to the client
            x = sine_wave()
            # p = generate_pos()
            p = [send_num1, send_num2, send_num3]
            # q = generate_quat()
            q = [sine_wave(), 3.14, -10, 35.0]
            msg_to_client = struct.pack(send_f_str, x, send_num2, send_num3, send_num4)
            # msg_to_client = struct.pack(send_f_str, x, p[0], p[1], p[2], q[0], q[1], q[2], q[3])
            conn.send(msg_to_client)

            # receive message from client
            msg_from_client = conn.recv(recv_num_floats * 4) # each float is 4 bytes
            recvd_floats = struct.unpack(recv_f_str, msg_from_client)
            # print("Recevied Floats: ", recvd_floats)

            time2 = time.time() # end timer

            # print frequency
            print("Freq. [hz]: {:.2f}, {}".format(1 / (time2 - time1), x))

    # soft shutdown if keyboard interrupt 
    except KeyboardInterrupt:

        print("\nKeyboard Interrupt. Closing Socket")
        conn.close()  # close the connection
    
    print("Server socket closed.")
    exit()

if __name__ == '__main__':
    
    server_program()

