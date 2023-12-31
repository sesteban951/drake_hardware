#!/usr/bin/env pthon

import rospy
from geometry_msgs.msg import PoseStamped
from pydrake.all import *
import numpy as np

class OptiTrack(LeafSystem):
    """
    Class used to subscirbe to OptiTrack data and interface with Drake
    """
    def __init__(self):
        # init Leaf system
        LeafSystem.__init__(self)

        # set vector output port
        self._cache = self.DeclareCacheEntry(description="controller output cache",
                                             value_producer=ValueProducer(
                                             allocate=lambda: AbstractValue.Make(dict()),
                                             calc=lambda context, output: output.set_value(
                                             self.CalcOutput(context))))
        
        # position quaternion output
        self.DeclareVectorOutputPort("position",
                                     BasicVector(3),
                                     lambda context, output: output.set_value(
                                     self._cache.Eval(context)["position"]),
                                     prerequisites_of_calc={self._cache.ticket()})
        # quaternion output
        self.DeclareVectorOutputPort("quaternion",
                                     BasicVector(4),
                                     lambda context, output: output.set_value(
                                     self._cache.Eval(context)["quaternion"]),
                                     prerequisites_of_calc={self._cache.ticket()})

        # values for output of this system
        self.pos  = np.zeros(3) 
        self.quat = np.zeros(4)

        self.wait_for_data = False # wait to get data from OptiTrack
        self.fresh_data = False    # flag for fresh data

        # ROS parameters
        node_name = 'optitrack_listener'
        topic_name = '/vrpn_client_node/adam/pose'
        hz = 300

        # setup ROS node
        rospy.init_node(node_name, anonymous=True)  # init ROS node
        self.sub = rospy.Subscriber(topic_name,     # subscibe to vrpn client node
                                    PoseStamped, 
                                    self.callback)
        self.rate = rospy.Rate(hz)                  # set loop rate to 30 Hz  (optional)   
        rospy.sleep(1)                              # wait for subscriber to connect

    # get pose and twist data from OptiTrack
    def callback(self,data):

        # set the local position and orientation variables
        pos_world = data.pose.position
        quat_world = data.pose.orientation

        self.pos = np.array([[pos_world.x], [pos_world.y], [pos_world.z]])
        self.quat = np.array([[quat_world.w],[quat_world.x], [quat_world.y], [quat_world.z]])
        
        # set fresh data flag to true
        self.fresh_data = True

    # compute output of this leaf system
    def CalcOutput(self,context):
        
        # wait until you get new data
        if self.wait_for_data:

            # loop until you get new data
            while not self.fresh_data:
                self.rate.sleep()
            
            # set fresh data flag to false
            self.fresh_data = False
        
        # return the position and quaternion
        pose_info = {"position": self.pos, 
                     "quaternion": self.quat}
        
        return pose_info
