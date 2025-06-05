#!/usr/bin/env python3
import utm
import json
import os
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseStamped
from gps_conv import GPS_utils
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class TargetTrajNode(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name=None) -> None:
        super().__init__(f"{target_name}_trajectory")
        self.client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not availabel, waiting...')
        
        self.target_name = target_name
        client_cb_grp = MutuallyExclusiveCallbackGroup()
        sub_cb_grp = MutuallyExclusiveCallbackGroup()
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.traj_callback)
        self.subscriber = self.create_subscription(PoseStamped, "/drone160", self.subscriber_callback, 1, callback_group=sub_cb_grp)
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.state_msg = EntityState()
        self.state_msg.name = self.target_name

        # self.state_msg.pose.position.x = self.begin_pose[0]
        # self.state_msg.pose.position.y = self.begin_pose[1]
        # self.state_msg.pose.position.z = self.begin_pose[2]
    
        #self.get_logger().info(f"set initial states for {self.target_name}")
        self.request = SetEntityState.Request()
    
    def subscriber_callback(self, data):
        self.x_pos = data.pose.position.x/1000.0
        self.y_pos = data.pose.position.y/1000.0
        self.z_pos = data.pose.position.z/1000.0


    def traj_callback(self): 

        self.state_msg.pose.position.x = float(self.x_pos)
        self.state_msg.pose.position.y = float(self.y_pos)
        self.state_msg.pose.position.z = float(self.z_pos)
        print(float(self.x_pos),float(self.y_pos),float(self.z_pos))
        _q = quaternion_from_euler(0, 0, 1)
        self.state_msg.pose.orientation.x = _q[0]
        self.state_msg.pose.orientation.y = _q[1]
        self.state_msg.pose.orientation.z = _q[2]
        self.state_msg.pose.orientation.w = _q[3]
        
        self.request.state = self.state_msg
        future = self.client.call_async(self.request)
        
        if future.done():
            print(future.done())
            response = future.result()
            self.get_logger().info("response: " + response)

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.get_global_executor()

    # data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    # with open(data_dir + "purdue.json", "r") as fstream:
    #     data = json.load(fstream)
    origin_lat = 40.427751
    origin_lon = -86.912004
    origin_alt = 0
    gps_conv = GPS_utils()
    gps_conv.setENUorigin(lat = origin_lat, lon = origin_lon, height = origin_alt)
    # # Lat/Lon from the datset 
    # lat =np.array(data['lat'])
    # lon = np.array(data['lon'])
    # gzb_x = []
    # gzb_y = []
    # for i in range(lat.shape[0]):
    #    enu = gps_conv.geo2enu(lat[i],lon[i],0)
    #    gzb_x.append(enu[0,0])
    #    gzb_y.append(enu[1,0])
    # gzb_x = np.array(gzb_x)
    # gzb_y = np.array(gzb_y)   

    # # Add heading and create a np array. 
    # car_data_heading = np.ones_like(gzb_x)
    # car_data_gzb = np.vstack([gzb_x, gzb_y]).T
    # car_data_gzb = np.vstack([car_data_gzb.T, car_data_heading]).T

    # # Initial position of the vehicle
    # x0_car = np.array([car_data_gzb[0,0], car_data_gzb[0,1], 1])

    traj = TargetTrajNode("laser_0")
    executor.add_node(traj)
    
    executor.spin()
    traj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
