#!/usr/bin/env python3
import json
import os
import time
from unittest import result
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetEntityState
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class StreamSkycamNode(Node):
    def __init__(self, sv_data, cno_max) -> None:
        super().__init__("satview_streamer")
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
       
        client_cb_grp = MutuallyExclusiveCallbackGroup()
        sub_cb_grp = MutuallyExclusiveCallbackGroup()
        pub_cb_grp = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(GetEntityState, "/get_entity_state", callback_group=client_cb_grp)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo get_entity_state service is not availabel, waiting...')
        self.subscriber = self.create_subscription(Image, "skycam/image_raw", self.subscriber_callback, 1, callback_group=sub_cb_grp)
        self.publisher = self.create_publisher(Image,"skycam/satview",1,callback_group = pub_cb_grp)
        self.sv_data = sv_data
        self.cno_max = cno_max
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.elapsed = 0
        self.time_last = self.get_clock().now()

    def send_request(self):
        self.request = GetEntityState.Request()
        self.request.name = 'laser_0'
        self.request.reference_frame = 'hong_kong'
        result = self.client.call(self.request)
        return result

    def subscriber_callback(self, data):
        result = self.send_request()
        model_pose = result.state
        model_euler = euler_from_quaternion(model_pose.pose.orientation)    
        # ENU (gzb) to NED
        heading = np.pi/2 - model_euler[2]
        cv_img = self.br.imgmsg_to_cv2(data, "bgr8")
        cv_img = np.array(np.flip(cv_img, axis=0))
        
        img_height = cv_img.shape[0]
        img_width = cv_img.shape[1]
        img_center = np.array([img_height/2.0, img_width/2.0]) # [250 250]
        
        r_max = np.min(img_center)
        green = (0, 255, 0)
        red = (0, 0, 255)
        blue = (255, 0, 0)
        
        self.elapsed += (self.get_clock().now()-self.time_last).nanoseconds / 1e9
        self.time_last = self.get_clock().now()
        
        for sv_id in self.sv_data.keys():
            elev = self.sv_data[sv_id]["mean"][0]
            azim = self.sv_data[sv_id]["mean"][1]

            index = int(self.elapsed*10 % len(self.sv_data[sv_id]["cno"]))
            cno = self.sv_data[sv_id]["cno"][index]

            r = (90.0 - elev)/90.0 * r_max
            theta = np.deg2rad(azim) - np.pi/2 - heading

            x = int(r*np.cos(theta) + img_center[0])
            y = int(r*np.sin(theta) + img_center[1])

            cv2.circle(cv_img, (x, y), 10, (0, int((cno)/self.cno_max*255), int((self.cno_max-cno)/self.cno_max*255)/2), -1)
            cv2.circle(cv_img, (x, y), 11, (0, 0, 255), 2)
            cv2.putText(cv_img, sv_id, (x-10, y-15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, green, 1)
        
        nesw = ["N", "E", "S", "W"]
        for i in range(4):
            theta = i*np.pi/2 - np.pi/2 - heading
            r = 235
            x = int(r*np.cos(theta) + img_center[0])
            y = int(r*np.sin(theta) + img_center[1])
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_img, nesw[i], (x,y), font, 0.5, green, 2)
        ros_img = self.br.cv2_to_imgmsg(cv_img, "bgr8")
        self.publisher.publish(ros_img)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    sv_filename = data_dir + "hk_data_sv_mean.json"
    sv_data = {}
    with open(sv_filename, "rb") as fstream:
        sv_data = json.load(fstream)
    cno_max = np.max([sv_data[key]["cno_max"] for key in sv_data.keys()])
    # Create the node
    stream_skycam_node = StreamSkycamNode(sv_data, cno_max)
    executor.add_node(stream_skycam_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        stream_skycam_node.destroy_node()
        # Shutdown the ROS client library for Python
        rclpy.shutdown()

if __name__ == '__main__':
    main()





