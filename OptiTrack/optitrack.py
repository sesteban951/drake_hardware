#!/usr/bin/env pthon

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np

class OptiTrack:
    """
    Class used to subscirbe to OptiTrack data and interface with Drake
    """
    def __init__(self):

        # stuff to send over drake system
        self.pos  = np.zeros(3)
        self.quat = np.zeros(4)

        # ROS parameters
        node_name = 'optitrack_listener'
        topic_name = '/vrpn_client_node/hopper/pose'
        hz = 30

        # setup ROS node
        rospy.init_node(node_name, anonymous=True)  # init ROS node
        self.sub = rospy.Subscriber(topic_name,     # subscibe to vrpn client node
                                    PoseStamped, 
                                    self.callback)
        self.rate = rospy.Rate(hz)                  # set loop rate to 30 Hz    
        rospy.sleep(1)                              # wait for subscriber to connect

    # get pose and twist data from OptiTrack
    def callback(self,data):

        # set the local position and orientation variables
        pos_world = data.pose.position
        quat_world = data.pose.orientation

        self.pos = np.array([[pos_world.x], [pos_world.y], [pos_world.z]])
        self.quat = np.array([[quat_world.x], [quat_world.y], [quat_world.z], [quat_world.w]])

    # main loop to execute
    def get_pose(self):

        while not rospy.is_shutdown():
            print("Position: ", self.pos)
            print("Orientation: ", self.quat)

            self.rate.sleep()

if __name__ == '__main__':
    opti = OptiTrack()
    opti.get_pose()

