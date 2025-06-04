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
from geometry_msgs.msg import Pose
from gps_conv import GPS_utils

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

def target_traj_gps(t, *args):
    car_data_gzb = args[0]
    index = int(t % car_data_gzb.shape[0])
    next_index = index + 1
    # Stacking the position, orientation and velocity of the model 
    pos = [car_data_gzb[index,0], car_data_gzb[index,1], 1]
    next_pos = [car_data_gzb[next_index,0], car_data_gzb[next_index,1], 1]
    att = np.array([0, 0, car_data_gzb[index, 2]])
    vel = (np.array(next_pos) - np.array(pos))
    return np.concatenate([pos, att, vel])

class TargetTrajNode(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name=None, traj_f=None, begin_pose=np.zeros(6, dtype=np.float64), *traj_args) -> None:
        super().__init__(f"{target_name}_trajectory")
        self.client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not availabel, waiting...')
        
        self.target_name = target_name
        self.traj_f = traj_f
        self.begin_pose = begin_pose
        self.traj_args = traj_args

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.traj_callback)
        self.elapsed = 0
        self.time_last = self.get_clock().now()

        self.get_logger().info(f"setting state for {self.target_name}")
        self.state_msg = EntityState()
        self.state_msg.name = self.target_name
        # state_msg.reference_frame = "world" # default frame

        self.state_msg.pose.position.x = self.begin_pose[0]
        self.state_msg.pose.position.y = self.begin_pose[1]
        self.state_msg.pose.position.z = self.begin_pose[2]
    
        self.get_logger().info(f"set initial states for {self.target_name}")
        self.request = SetEntityState.Request()

    def traj_callback(self):
        self.elapsed += (self.get_clock().now()-self.time_last).nanoseconds / 1e9
        self.time_last = self.get_clock().now()

        _pose = self.traj_f(self.elapsed, *self.traj_args)
        self.state_msg.pose.position.x = _pose[0]
        self.state_msg.pose.position.y = _pose[1]
        self.state_msg.pose.position.z = _pose[2]

        _q = quaternion_from_euler(_pose[3], _pose[4], _pose[5])
        self.state_msg.pose.orientation.x = _q[0]
        self.state_msg.pose.orientation.y = _q[1]
        self.state_msg.pose.orientation.z = _q[2]
        self.state_msg.pose.orientation.w = _q[3]
        
        self.state_msg.twist.linear.x = _pose[6]
        self.state_msg.twist.linear.y = _pose[7]
        self.state_msg.twist.linear.z = _pose[8]
        self.request.state = self.state_msg
        future = self.client.call_async(self.request)
        
        if future.done():
            print(future.done())
            response = future.result()
            #self.get_logger().info("response: " + response)

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.get_global_executor()

    start = -40
    end = 40
    num_points = int(abs(end-start +1)/5)
    car_data_x = np.linspace(start,end, num_points)
    car_data_y = np.linspace(start,end,num_points)
    car_data_gzb = np.zeros((num_points*num_points, 3))
    count = 0
    for i in range(num_points):
        car_data_gzb[count, 0] = car_data_x[i]
        count+=1          

    # Initial position of the vehicle
    x0_car = np.array([car_data_gzb[0,0], car_data_gzb[0,1], 1])

    traj = TargetTrajNode("laser_0", target_traj_gps, x0_car, car_data_gzb)
    executor.add_node(traj)
    
    executor.spin()
    traj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
