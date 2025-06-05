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
from gnss_multipath_plugin.msg import MultipathOffset
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from multipath_sim.kalman_filter import KalmanFilter
from multipath_sim.sat_pos import get_sat_position_ecef
from multipath_sim.coord import conv_latlon_to_gzb, ecef2geodetic, conv_gzb_to_latlon, geodetic2ecef
from multipath_sim.trialteration import Trilateration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from multipath_sim.laika.astro_dog import AstroDog
from multipath_sim.laika.helpers import get_el_az

def get_ranges(sat_ecef, rec_ecef):
    ranges = []
    for i in range(sat_ecef.shape[0]):
        ranges.append(np.linalg.norm(sat_ecef[i] - rec_ecef))
    return np.array(ranges)

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

class BroadcastGPSNode(Node):
    def __init__(self, nav_data) -> None:
        super().__init__("broadcast_pos")
        client_cb_grp = MutuallyExclusiveCallbackGroup()
        sub_cb_grp = MutuallyExclusiveCallbackGroup()
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscriber = self.create_subscription(
        MultipathOffset, 
        "/multipath/offset", 
        self.offset_callback, 
        10, callback_group=sub_cb_grp)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.nav_data= nav_data
        self.client = self.create_client(GetEntityState, "/get_entity_state", callback_group=client_cb_grp)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo get_entity_state service is not availabel, waiting...')
        self.true_pos_list =[]
        self.gps_pos_list = []
        self.astrodog = AstroDog(valid_const = ['GPS'])

    def send_request(self):
        self.request = GetEntityState.Request()
        self.request.name = 'laser_0'
        self.request.reference_frame = 'hong_kong'
        result = self.client.call(self.request)
        return result
    
    def offset_callback(self, data):    
        # True position obtained from Gazebo
        result = self.send_request()
        true_pose = result.state
        true_pos = [true_pose.pose.position.x, true_pose.pose.position.y, true_pose.pose.position.z]

        # Lat/Lon bounds for the segment of the map in gazebo     
        lat_bound = np.array([22.3066, 22.2903])
        lon_bound = np.array([114.171, 114.188])

        # Applying correction in the offset to align the origin
        origin_offset = np.array([77, 15])
        
        # Get the Lat/Lon from ENU (gzb world)
        lat, lon = conv_gzb_to_latlon(true_pos[0],true_pos[1], lat_bound, lon_bound, origin_offset)
        rec_ecef = geodetic2ecef([lat, lon, 5])

        #ecef_x, ecef_y, ecef_z = conv_lla_to_ecef(lat, lon, 0)
        sats = ['G01','G04','G07','G08', 'G09','G16','G21', 'G27']
        sats_vis = []
        sats_vis_idx = []
        multipath_offset = []
        # Check for the visible satellite
        for i,sat in enumerate(sats):
            if data.sats_blocked[i]!=1 and data.range_offset[i] < 100:
                sats_vis.append(sat)
                sats_vis_idx.append(float(i))
                multipath_offset.append(data.range_offset[i])
        multipath_offset = np.array(multipath_offset)
        #self.get_logger().info('{}'.format(multipath_offset))
        gps_pos_fix_gzb = np.empty(3)
        gps_pos_fix_gzb[0] = np.nan
        gps_pos_fix_gzb[1] = np.nan
        gps_pos_fix_gzb[2] = np.nan

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "laser_0"
        t.transform.translation.x = true_pose.pose.position.x
        t.transform.translation.y = true_pose.pose.position.y
        t.transform.translation.z = true_pose.pose.position.z
        t.transform.rotation.x = true_pose.pose.orientation.x
        t.transform.rotation.y = true_pose.pose.orientation.y
        t.transform.rotation.z = true_pose.pose.orientation.z
        t.transform.rotation.w = true_pose.pose.orientation.w
        self.br.sendTransform(t)
        
        #self.get_logger().info('Visible Sat:{}'.format(len(sats_vis)))
        if len(sats_vis) > 5 :  
            sat_pos = get_sat_position_ecef(self.nav_data, sats_vis)    
            ranges = get_ranges(sat_pos, rec_ecef) 
            ranges += multipath_offset
            print(sat_pos, ranges)
            TriGPS = Trilateration(sat_pos, np.array(ranges))
            rec_pos, _ = TriGPS.optimize()
            pos_geodetic = ecef2geodetic(rec_pos)
            pos_gzb = conv_latlon_to_gzb(pos_geodetic[0], pos_geodetic[1], lat_bound, lon_bound, origin_offset)
            gps_pos_fix_gzb[0] = pos_gzb[0]
            gps_pos_fix_gzb[1] = pos_gzb[1]
            gps_pos_fix_gzb[2] = 0
            if not(math.isnan(gps_pos_fix_gzb[0]) or math.isnan(gps_pos_fix_gzb[1]) or math.isnan(gps_pos_fix_gzb[1])): 
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "map"
                t.child_frame_id = "gps_fix"
                t.transform.translation.x = gps_pos_fix_gzb[0]
                t.transform.translation.y = gps_pos_fix_gzb[1]
                t.transform.translation.z = gps_pos_fix_gzb[2]
                t.transform.rotation.x = true_pose.pose.orientation.x
                t.transform.rotation.y = true_pose.pose.orientation.y
                t.transform.rotation.z = true_pose.pose.orientation.z
                t.transform.rotation.w = true_pose.pose.orientation.w
                self.br.sendTransform(t)
        self.true_pos_list.append(true_pos)
        self.gps_pos_list.append(gps_pos_fix_gzb)
    
    def log_pos_data(self, data_dir):
        with open(data_dir + "pos_data_log.json", "w") as fs:
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
    nav_file = data_dir + '/hksc135c.21n'
    nav_data = gr.rinexnav(nav_file, use='G')
    # Create the node
    broadcast_gps_node = BroadcastGPSNode(nav_data)
    executor.add_node(broadcast_gps_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        broadcast_gps_node.log_pos_data(data_dir)
        broadcast_gps_node.get_logger().info('Logging complete!')
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        broadcast_gps_node.destroy_node()
        
        # Shutdown the ROS client library for Python
        rclpy.shutdown()

if __name__ == '__main__':
    main()
