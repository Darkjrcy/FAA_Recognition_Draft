#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry

import json
import matplotlib.pyplot as plt
import roslib
roslib.load_manifest("multipath_sim")
import rospy
from multipath_sim.msg import MultipathOffset
import os

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/hk_data" 

def cb_fn(data: MultipathOffset, cb_args):
    offset = cb_args[0]
    gps_bias = cb_args[1]
    gps_noise = cb_args[2]
    true_position = cb_args[3]
    num_block_sat = cb_args[4]
    num_vis_sat  = cb_args[5]

    offset_msg = data
    offset.append(offset_msg.offset)
    gps_bias.append(offset_msg.gps_bias)
    gps_noise.append(offset_msg.gps_noise)
    true_position.append(offset_msg.true_position)
    num_block_sat.append(offset_msg.num_block_sat)
    num_vis_sat.append(offset_msg.num_vis_sat)


def main():
    rospy.init_node("record_heat_map")

    offset = []
    gps_bias = []
    gps_noise = []
    true_position = []
    num_block_sat = []
    num_vis_sat = []

    callback_args = [offset, gps_bias, gps_noise, true_position, num_block_sat, num_vis_sat]
    offset_sub = rospy.Subscriber("/multipath/offset", MultipathOffset, callback=cb_fn, callback_args=callback_args)

    try:
        rospy.spin()
        with open(data_dir+"_heatmap.json", "w") as fs:
            offset = np.array(offset)
            gps_bias = np.array(gps_bias)
            gps_noise = np.array(gps_noise)
            true_position = np.array(true_position)
            num_block_sat = np.array(num_block_sat)
            num_vis_sat = np.array(num_vis_sat)
            data = {"aff_x": (true_position[:,0]+offset[:,0]).tolist(),
                "aff_y": (true_position[:,1]+offset[:,1]).tolist(), 
                "offset_x": offset[:,0].tolist(),  
                "offset_y": offset[:,1].tolist(),
                "true_x": true_position[:,0].tolist(),
                "true_y": true_position[:,1].tolist(), 
                "gps_bias_x": gps_bias[:,0].tolist(),
                "gps_bias_y": gps_bias[:,1].tolist(),
                "gps_noise_x": gps_noise[:,0].tolist(),
                "gps_noise_y": gps_noise[:,1].tolist(), 
                "num_vis_sat": num_vis_sat.tolist(),
                "num_block_sat": num_block_sat.tolist(),
                }
            json.dump(data, fs)

    except KeyboardInterrupt:
        print("shutting down...")

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("ros interrupt")
    