#!/usr/bin/env python3
import json
import numpy as np
from numpy.core.function_base import geomspace
from numpy.core.numeric import NaN
import roslib
import tf2_ros
roslib.load_manifest("multipath_sim")
import rospy
from multipath_sim.msg import MultipathOffset

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import os

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/hk_data" 

def cb_fn(data, data_list):
    data_list.append([data.offset[0], data.offset[1], data.offset[2]])
    pass

def main():
    rospy.init_node("random_walk")
    data=[]
    offset_sub = rospy.Subscriber("/multipath/offset", MultipathOffset, callback=cb_fn, callback_args= data)
    try:
        rospy.spin()
        with open(data_dir+"noise.json", "w") as fs:
            data = np.array(data)
            data = {"x": data[:,0].tolist(),"y": data[:,1].tolist(), "z": data[:,2].tolist() }
            json.dump(data, fs)

    except KeyboardInterrupt:
        print("shutting down...")

if __name__=="__main__":
    main()
