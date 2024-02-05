
"""
Example script that communicates to Teensy over serial
  1) Connect the Teensy to your computer via USB and note the tty port (e.g., /dev/ttyACM0)
  2) Ensure that the data packet sizes are agreed upon before running the script
  3) In a terminal: roslaunch vrpn_client_ros sample.launch server:=<IPV4_of_optitrack_computer>
  4) Make sure that your rigid body name "topic_name" matches the one from motive in this script
  5) Launch the script and observe the data being sent and received
  6) Run the Arduino code on the Teensy
By: Sergio Esteban (sesteban@caltech.edu)
"""

import tty    # Used to set the tty port to raw mode
import struct # Used to pack and unpack floats as bytes

# ROS imports
import rospy  
from geometry_msgs.msg import PoseStamped

import time
import numpy as np

class SerialNode():
    """
    SerialNode class that get pose info and sends it to the Teensy over Serial.
    Also receives data from the Teensy over Serial.
    """

    def __init__(self, tty_port):

        # Replace '/dev/ttyACM0' with the appropriate serial port for your Teensy
        self.tty_port = tty_port

        # tty file descriptor
        self.tty_fd = open(tty_port, 'wb+', buffering=0)
        tty.setraw(self.tty_fd)  # Set the tty port to raw mode

        # containers for the send data
        self.send_data = [0.0, 0.0, 0.0,      # Position
                          0.0, 0.0, 0.0, 0.0, # Quaternion, (qw, qx, qy, qz)
                          0.0, 0.0,           # ESC right, left
                          0.0, 0.0, 0.0,      # Right Leg
                          0.0, 0.0, 0.0]      # Left Leg
        self.num_float_send = len(self.send_data)  # number of floats to send
        self.pos = np.array([0.0, 0.0, 0.0])
        self.quat = np.array([0.0, 0.0, 0.0, 0.0])
        self.esc = np.array([0.0, 0.0])
        self.right_leg_send = np.array([0.0, 0.0, 0.0])
        self.left_leg_send = np.array([0.0, 0.0, 0.0])

        # containers for the receive data
        self.recv_data = [0.0, 0.0, 0.0, # Right Leg 
                          0.0, 0.0, 0.0] # Left Leg
        self.num_float_recv = len(self.recv_data)
        self.right_leg_recv = np.array([0.0, 0.0, 0.0])
        self.left_leg_recv = np.array([0.0, 0.0, 0.0])

        # ROS parameters
        node_name = 'optitrack_listener'
        topic_name = '/vrpn_client_node/hopper/pose'
        hz = 300

        # setup ROS node
        rospy.init_node(node_name, anonymous=True)  # init ROS node
        self.sub = rospy.Subscriber(topic_name,     # subscibe to vrpn client node
                                    PoseStamped, 
                                    self.callback)
        self.rate = rospy.Rate(hz)                  # set loop rate [hz]    

        # FOR TESTING:
        # self.current_optitrack_pos = None
        # self.current_teensy_pos = None

        # self.t0_opti = time.time()
        # self.t0_teen = time.time()

        rospy.sleep(1)                              # wait for subscriber to connect

    # get pose and twist data from OptiTrack
    def callback(self,data):

        # set the local position and orientation variables
        pos_world = data.pose.position
        quat_world = data.pose.orientation

        # FOR TESTING:
        # if current_optitrack is None, set it to the current pose
        # if self.current_optitrack_pos is None:
        #     self.current_optitrack_pos = np.array([pos_world.x, pos_world.y, pos_world.z])
    
        # if np.all(self.current_optitrack_pos != np.array([pos_world.x, pos_world.y, pos_world.z])):
        #     self.current_optitrack_pos =  np.array([pos_world.x, pos_world.y, pos_world.z])
        #     dt = time.time() - self.t0_opti
        #     self.t0_opti = time.time()
        #     print(f"optitrack_dt: {dt}")  # TESTED: 10 ms max delay between new updates (100 Hz)
            
        self.pos = np.array([[pos_world.x], [pos_world.y], [pos_world.z]])
        self.quat = np.array([[quat_world.w], [quat_world.x], [quat_world.y], [quat_world.z]])
        self.esc[0] = self.sine_wave(0)
        self.esc[1] = self.sine_wave(np.pi/4)
        self.right_leg_send[0] = self.sine_wave(0)
        self.right_leg_send[1] = self.sine_wave(0)
        self.right_leg_send[2] = self.sine_wave(0)
        self.left_leg_send[0] = self.sine_wave(np.pi/4)
        self.left_leg_send[1] = self.sine_wave(np.pi/4)
        self.left_leg_send[2] = self.sine_wave(np.pi/4)

    # arbitrary function to generate data
    def sine_wave(self, phi):
        t = time.time()
        f = 0.5
        x = 35.0 * np.sin(2 * np.pi * f * t + phi)
        return  x

    def send_floats(self):
        """
        Sends a list of floats to the Teensy over Serial
        """
        # Pack the floats as bytes and send
        for value in self.send_data:
            self.tty_fd.write(struct.pack('f', value))
            self.tty_fd.flush()

    def receive_floats(self):
        """
        Receives a list of floats from the Teensy over Serial
        """
        # Read the bytes and unpack as floats
        data_bytes = self.tty_fd.read(4 * self.num_float_recv)
        self.recv_data = struct.unpack('f' * self.num_float_recv, data_bytes)
        self.right_leg_recv = np.array([self.recv_data[0], self.recv_data[1], self.recv_data[2]])
        self.left_leg_recv = np.array([self.recv_data[3], self.recv_data[4], self.recv_data[5]])

    def stream_serial(self):

        # main loop
        while not rospy.is_shutdown():

            # FOR TESTING:
            # d0 = time.time()

            # send floats to teensy
            self.send_data = [self.pos[0], self.pos[1], self.pos[2], 
                              self.quat[0], self.quat[1], self.quat[2], self.quat[3],
                              self.esc[0], self.esc[1],
                              self.right_leg_send[0], self.right_leg_send[1], self.right_leg_send[2],
                              self.left_leg_send[0], self.left_leg_send[1], self.left_leg_send[2]]
            self.send_floats()

            # Receive multiple floats from Teensy
            self.receive_floats()
            self.right_leg_recv = np.array([self.recv_data[0], self.recv_data[1], self.recv_data[2]])
            self.left_leg_recv = np.array([self.recv_data[3], self.recv_data[4], self.recv_data[5]])

            # FOR TESTING:
            # current_teensy_pos = np.array([self.recv_data[0], self.recv_data[1], self.recv_data[2]])
            # err = np.linalg.norm(self.current_optitrack_pos - current_teensy_pos)
            # if err < 0.05:
            #     dt = time.time() - self.t0_opti
            #     print(f"teensy_dt: {dt}")  # TESTED: for 0.05 tol, we get max 9 ms lag (110 Hz)
            # print(time.time() - d0) # TESTED: raw transfer rate: 0.4 ms delay (2.5 kHz)

            # ROS sleep
            self.rate.sleep()

        # safe shutdown
        print("\n\nCommunication terminated by user.")
        self.tty_fd.close()
        print("TTY port closed.")

#######################################################################################################33
            
if __name__ == '__main__':
    # Create a SerialNode object
    tty_port = '/dev/ttyACM0'
    serial_node = SerialNode(tty_port)

    # Stream the serial communication
    serial_node.stream_serial()
    
