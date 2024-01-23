# example of a python server that communicates with a C++ client
# 0. Connect your PC to the wifi router  via ethernet.
# 1. In a terminal: roslaunch vrpn_client_ros sample.launch server:=<IPV4_of_optitrack_computer>
# 2. Make sure that your rigid body name "topic_name" matches the one from motive in this script
# 3. Launch this python script via: python3 py_optitrack_server.py
# 4. Run the arduino code (ensure to pay attention to the instructiosn there too. I.e., SSID, PW, INET)
# By: Sergio Esteban (sesteban@caltech.edu)

import socket # to use socket prgramming
import struct # to use structs  when sending data
import time   # to watch the communication frequency
import numpy as np

# import threading        # might need these to "multi thread" <-- one GIL 
# import multiprocessing  # might need these to "multi thread" <-- multiple GILs

import rospy
from geometry_msgs.msg import PoseStamped

#############################################################################################
class OptiTrackServer:
    """
    Class used to subscirbe to OptiTrack data and interface with Drake
    """
    def __init__(self):

        self.t0 = time.time()

        # send and receive variables
        self.send_num_floats = 9
        self.send_f_str = 'f' * self.send_num_floats # number of f's is number of floats

        self.recv_num_floats = 6
        self.recv_f_str= 'f' * self.recv_num_floats

        # stuff to send over drake system
        self.esc  = np.zeros(2)  # right and left ESC
        self.pos  = np.zeros(3)  # position x, y, z
        self.quat = np.zeros(4)  # quaternion, qw, qx, qy, qz

        # ROS parameters
        node_name = 'optitrack_listener'
        topic_name = '/vrpn_client_node/drone/pose'
        hz = 300

        # setup ROS node
        rospy.init_node(node_name, anonymous=True)  # init ROS node
        self.sub = rospy.Subscriber(topic_name,     # subscibe to vrpn client node
                                    PoseStamped, 
                                    self.callback)
        self.rate = rospy.Rate(hz)                  # set loop rate [hz]    
        rospy.sleep(1)                              # wait for subscriber to connect


    # get pose and twist data from OptiTrack
    def callback(self,data):

        # set the local position and orientation variables
        pos_world = data.pose.position
        quat_world = data.pose.orientation

        self.esc[0] = self.sine_wave(0)
        self.esc[1] = self.sine_wave(np.pi/4)
        self.pos = np.array([[pos_world.x], [pos_world.y], [pos_world.z]])
        self.quat = np.array([[quat_world.w], [quat_world.x], [quat_world.y], [quat_world.z]])
    
    # arbitrary function to generate data
    def sine_wave(self, phi):
        t = time.time() - self.t0
        f = 0.5
        x = 35.0 * np.sin(2 * np.pi * f * t + phi)
        return  x

    # setup all the networking
    def setup_server_socket(self):

        # network variables
        host = ''    # get the hostname, in my case, the machine name (if on same PC)
        port = 8081  # initiate port no above 1024, can be anything

        # SOCK_STREAM is TCP and SOCK_DGRAM is UDP
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create the server endpoint for commnuication
        print("Server socket created")
        
        # setup server socket options
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # bind host address and port together which makes a unique connection
        server_socket.bind((host, port))  
        print("Server socket binded: Host [{}], Port: [{}] ".format(host, port))
        print("Waiting for connection...")

        # assign 1 client to be connected at a time, and wait for client connection
        server_socket.listen(1)                      
        # accept new connection when client connects
        self.conn, address = server_socket.accept()  
        print("Connection from: " + str(address))

    # main loop to execute
    def server_program(self):

        # setup the server socket
        self.setup_server_socket()

        # main loop
        while not rospy.is_shutdown():

            # send message to the client
            msg_to_client = struct.pack(self.send_f_str,
                                        self.esc[0],  self.esc[1],
                                        self.pos[0],  self.pos[1],  self.pos[2],
                                        self.quat[0], self.quat[1], self.quat[2], self.quat[3])
            self.conn.send(msg_to_client)

            # receive message from client
            msg_from_client = self.conn.recv(self.recv_num_floats * 4)     # each float is 4 bytes
            recvd_floats = struct.unpack(self.recv_f_str, msg_from_client)

            # print the received message
            print("Received: ", recvd_floats)

            self.rate.sleep()

#############################################################################################

if __name__ == '__main__':
    
    opti_serve = OptiTrackServer()
    opti_serve.server_program()

