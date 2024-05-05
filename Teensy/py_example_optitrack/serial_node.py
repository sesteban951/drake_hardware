"""
Example script that communicates to Teensy over serial (this is OptiTrack send only).

  1) Connect the Teensy to your computer via USB and note the tty port (e.g., /dev/ttyACM0).
  2) Ensure that the data packet sizes are agreed upon (on this script and arduino script) before running this.
  3) In a terminal: roslaunch vrpn_client_ros sample.launch server:=<IPV4_of_optitrack_computer>
  4) Make sure that your rigid body name "topic_name" matches the one from motive in this script. I.e,
     if your rigid body name is "harpy", then the topic_name should be '/vrpn_client_node/harpy/pose'
  5) Launch the script and observe the data being sent and received
  6) Run the Arduino code on the Teensy

By: Sergio Esteban (sesteban@caltech.edu)
"""

# Serial imports
import tty    # Used to set the tty port to raw mode
import struct # Used to pack and unpack floats as bytes

# ROS imports
import rospy  
from geometry_msgs.msg import PoseStamped

# standard imports
import numpy as np

class SerialNode():
    """
    SerialNode class that gets pose info and sends it to the Teensy over Serial.
    Also receives data from the Teensy over Serial.
    """

    def __init__(self, tty_port, hz):

        # Replace '/dev/ttyACM0' with the appropriate serial port for your Teensy
        self.tty_port = tty_port

        # tty file descriptor
        self.tty_fd = open(tty_port, 'wb+', buffering=0)
        tty.setraw(self.tty_fd)  # Set the tty port to raw mode

        # containers for the send data
        self.send_data = [0.0, 0.0, 0.0,          # Position
                          0.0, 0.0, 0.0, 0.0]     # Quaternion, (qw, qx, qy, qz)
        self.num_float_send = len(self.send_data) # number of floats to send
        
        # containers for the pose data to send
        self.pos = np.array([0.0, 0.0, 0.0])
        self.quat = np.array([0.0, 0.0, 0.0, 0.0])

        # ROS parameters
        node_name = 'optitrack_listener'
        topic_name = '/vrpn_client_node/harpy/pose'

        # setup ROS node
        rospy.init_node(node_name, anonymous=True)  # init ROS node
        self.sub = rospy.Subscriber(topic_name,     # subscibe to vrpn client node
                                    PoseStamped, 
                                    self.callback)
        self.rate = rospy.Rate(hz)                  # set loop rate [hz]    
        rospy.sleep(1)                              # wait for subscriber to connect

    # get pose and twist data from OptiTrack
    def callback(self,data):

        # set the position and orientation variables
        pos_world = data.pose.position
        quat_world = data.pose.orientation
        
        # set the position and orientation variables
        self.pos = np.array([[pos_world.x], [pos_world.y], [pos_world.z]])
        self.quat = np.array([[quat_world.w], [quat_world.x], [quat_world.y], [quat_world.z]])

    def send_floats(self):
        """
        Sends a list of floats to the Teensy over Serial
        """
        # Pack the floats as bytes and send
        for value in self.send_data:
            self.tty_fd.write(struct.pack('f', value))
            self.tty_fd.flush()

    def stream_serial(self):

        # main loop
        while not rospy.is_shutdown():

            # send floats to teensy
            self.send_data = [self.pos[0], self.pos[1], self.pos[2],                   # Position, (x, y, z)
                              self.quat[0], self.quat[1], self.quat[2], self.quat[3]]  # Quaternion, (qw, qx, qy, qz)
            self.send_floats()

            # ROS sleep to maintain desired loop rate
            self.rate.sleep()

        # safe shutdown
        print("\n\nCommunication terminated by user.")
        self.tty_fd.close()
        print("TTY port closed.")

#######################################################################################################33
            
if __name__ == '__main__':

    # Create a SerialNode object
    hz = 250                   # Desired Serial loop rate, [Hz]
    tty_port = '/dev/ttyACM0'  # USB port for Teensy
    serial_node = SerialNode(tty_port, hz)

    # Stream the serial communication
    serial_node.stream_serial()
    