#!/usr/bin/env python3
import json
import os
import time
from unittest import result
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class UBloxDataProcNode(Node):
    def __init__(self) -> None:
        super().__init__("ublox_data_processing")
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
       
        sub_cb_grp = MutuallyExclusiveCallbackGroup()
        self.subscriber = self.create_subscription(NavSatFix, "/fix", self.subscriber_callback, 1, callback_group=sub_cb_grp)
        self.lat_list =[]
        self.lon_list =[]
        self.count =0

    def subscriber_callback(self, data):
        self.count+=1
        if (self.count%10==0):

            self.get_logger().info('10 Messages logged')
        self.lat_list.append(data.latitude)
        self.lon_list.append(data.longitude) 

    def dump_data(self, data_dir, fname):
        with open(data_dir + fname + ".json", "w") as fs:
            lat = np.array(self.lat_list)
            lon = np.array(self.lon_list)
            data = {"lat": lat.tolist(),
                "lon": lon.tolist()
                }
            json.dump(data, fs)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    ubx_data_proc_node = UBloxDataProcNode()
    executor.add_node(ubx_data_proc_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        ubx_data_proc_node.dump_data(data_dir, "purdue")
        ubx_data_proc_node.get_logger().info('Logging complete!')
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        ubx_data_proc_node.destroy_node()
        # Shutdown the ROS client library for Python
        rclpy.shutdown()

if __name__ == '__main__':
    main()
