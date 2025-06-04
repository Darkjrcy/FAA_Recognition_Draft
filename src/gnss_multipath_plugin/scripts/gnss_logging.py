#!/usr/bin/env python3
import json
import os
import rclpy
import math
import georinex as gr
import numpy as np
import tf2_ros
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from numpy.core.numeric import NaN
from gnss_multipath_plugin.msg import GNSSMultipathFix
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup



class GNSSDataLogging(Node):
    def __init__(self) -> None:
        super().__init__("gnss_data_logging")
        client_cb_grp = MutuallyExclusiveCallbackGroup()
        sub_cb_grp = MutuallyExclusiveCallbackGroup()
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscriber = self.create_subscription(
        GNSSMultipathFix, 
        "/gnss_multipath_plugin/gnss_multipath_fix", 
        self.gnss_mulitpath_fix_callback, 
        10, callback_group=sub_cb_grp)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.true_pos_list =[]
        self.gps_pos_list = []
    
    def gnss_mulitpath_fix_callback(self, data):    
        # True position obtained from Gazebo
        
        enu_true = data.enu_true
        enu_gnss_fix = data.enu_gnss_fix
        self.true_pos_list.append(enu_true)
        self.gps_pos_list.append(enu_gnss_fix)

        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "map"
        # t.child_frame_id = "laser_0"
        # t.transform.translation.x = true_pose.pose.position.x
        # t.transform.translation.y = true_pose.pose.position.y
        # t.transform.translation.z = true_pose.pose.position.z
        # t.transform.rotation.x = true_pose.pose.orientation.x
        # t.transform.rotation.y = true_pose.pose.orientation.y
        # t.transform.rotation.z = true_pose.pose.orientation.z
        # t.transform.rotation.w = true_pose.pose.orientation.w
        # self.br.sendTransform(t)
    
    def log_pos_data(self, data_dir):
        with open(data_dir + "pos_data_log_heatmap.json", "w") as fs:
            true_pos = np.array(self.true_pos_list)
            gps_pos = np.array(self.gps_pos_list)
            data = {"true_pos": true_pos.tolist(),
                "gps_pos": gps_pos.tolist()
                }
            json.dump(data, fs)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    # Create the node
    gnss_data_logging_node = GNSSDataLogging()
    executor.add_node(gnss_data_logging_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        gnss_data_logging_node.log_pos_data(data_dir)
        gnss_data_logging_node.get_logger().info('Logging complete!')
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        gnss_data_logging_node.destroy_node()
        
        # Shutdown the ROS client library for Python
        rclpy.shutdown()

if __name__ == '__main__':
    main()
