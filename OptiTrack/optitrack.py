#!/usr/bin/env pthon

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np

class OptiTrack:
    """
    Class used to subscirbe to OptiTrack data
    """
    def __init__(self):

        # stuff to send over drake system
        self.quat = np.zeros(4)
        self.pos  = np.zeros(3)

        # ROS stuff
        rospy.init_node('optitrack_listener', anonymous=True)
        self.sub = rospy.Subscriber("/vrpn_client_node/hopper/pose", PoseStamped, self.callback)
        self.rate = rospy.Rate(30) 
        rospy.sleep(1)

    # send the pose and twist over drake system
    def callback(self,data):
        # log the pose and twist data
        pos_world = data.pose.position
        quat_world = data.pose.orientation

        self.pos = np.array([[pos_world.x], [pos_world.y], [pos_world.z]])
        self.quat = np.array([[quat_world.x], [quat_world.y], [quat_world.z], [quat_world.w]])

    # main loop to execute
    def get_pose(self):

        print("Starting OptiTrack Listener:")
        while not rospy.is_shutdown():
            print("Position: ", self.pos)
            print("Orientation: ", self.quat)
 
            print("\n")
            self.rate.sleep()

if __name__ == '__main__':
    opti = OptiTrack()
    opti.get_pose()

