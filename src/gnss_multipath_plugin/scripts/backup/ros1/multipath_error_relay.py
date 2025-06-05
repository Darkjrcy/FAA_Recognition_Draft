#!/usr/bin/env python3
from re import M
import numpy as np
import roslib
import tf2_ros
roslib.load_manifest("multipath_sim")
import rospy
import os
import utm
import json
import rospy
import numpy as np
import georinex as gr
import math
from numpy.core.numeric import NaN
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from tf.transformations import quaternion_from_euler
from multipath_sim.msg import MultipathOffset
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from multipath_sim.kalman_filter import KalmanFilter
from multipath_plugin.sat_pos import get_sat_position_ecef
from multipath_plugin.coord import conv_latlon_to_gzb, ecef2geodetic, conv_gzb_to_latlon, geodetic2ecef
from multipath_plugin.trialteration import Trilateration

def get_ranges(sat_ecef, rec_ecef):
    ranges = []
    for i in range(sat_ecef.shape[0]):
        ranges.append(np.linalg.norm(sat_ecef[i] - rec_ecef))
    return np.array(ranges)

def cb_fn(data, cb_args):
    # Callback args
    get_model_pose = cb_args[0]
    br = cb_args[1]
    odom_pub = cb_args[2]
    nav_data = cb_args[3]
    dop_pub = cb_args[4]
    gps_bias = cb_args[5]
    kf = cb_args[6]
    
    # # Gaussian random walk constant 
    # gps_corellation_time = 0.05
    

    current_time = rospy.Time.now()
    
    # True position obtained from Gazebo
    true_pose = get_model_pose("laser_0", "map")
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
    affected_pos = np.empty(3)
    affected_pos[0] = np.nan
    affected_pos[1] = np.nan
    affected_pos[2] = np.nan
    print(len(sats_vis))
    t = TransformStamped()
    t.header.stamp = current_time
    t.header.frame_id = "map"
    t.child_frame_id = "laser_0"
    t.transform.translation.x = true_pose.pose.position.x
    t.transform.translation.y = true_pose.pose.position.y
    t.transform.translation.z = true_pose.pose.position.z
    t.transform.rotation.x = true_pose.pose.orientation.x
    t.transform.rotation.y = true_pose.pose.orientation.y
    t.transform.rotation.z = true_pose.pose.orientation.z
    t.transform.rotation.w = true_pose.pose.orientation.w
    br.sendTransform(t)
    if len(sats_vis) > 4 :  
        sat_pos = get_sat_position_ecef(nav_data, sats_vis)    
        ranges = get_ranges(sat_pos, rec_ecef) 
        ranges += multipath_offset
        TriGPS = Trilateration(sat_pos, np.array(ranges))
        rec_pos, _ = TriGPS.optimize()
        pos_geodetic = ecef2geodetic(rec_pos)
        pos_gzb = conv_latlon_to_gzb(pos_geodetic[0], pos_geodetic[1], lat_bound, lon_bound, origin_offset)
        affected_pos[0] = pos_gzb[0]
        affected_pos[1] = pos_gzb[1]
        affected_pos[2] = 0

        #print(affected_pos, true_pos)
    # # Compute the DoP
    # dop = []
    # if (len(sats_vis_idx)>3):
    #     dop = compute_dop([ecef_x, ecef_y, ecef_z], [lat, lon, 0], sat_pos, sats_vis)
    #     dop_msg = Float64MultiArray()
    #     dop_msg.data = dop + sats_vis_idx  
    #     dop_pub.publish(dop_msg)
    # affected_pos = np.empty((3))

    
    # if (len(sats_vis)>3):
    #     # gps bias integration
    #     random_walk_mu = np.zeros(3)
    #     random_walk_gps_ = np.sqrt(dt) * dop[3] * np.random.normal(0, 1, 3)
    #     gps_bias +=  random_walk_gps_ - dt * (random_walk_mu + gps_bias)/ gps_corellation_time   
    #     affected_pos = true_pos + 0.5*np.array(data.offset) + gps_bias
    # else :
    #     gps_bias =  np.zeros(3) 
    #     affected_pos[:] = np.nan  
    #if (len(sats_vis)>3):
    #     # gps bias integration
    #     random_walk_mu = np.zeros(3)
    #     random_walk_gps_ = np.sqrt(dt) * np.random.normal(0, 1, 3)
    #     gps_bias +=  random_walk_gps_ - dt * (random_walk_mu + gps_bias)/ gps_corellation_time
    #     affected_pos = true_pos + gps_bias + 1 * np.array(data.offset)
    #     if reset_gps_bias == True:
    #         gps_bias = np.zeros(3)
    #         reset_gps_bias = False
    # else :
    #     affected_pos[:2] = np.nan  
    #     reset_gps_bias = True
    #     #random_walk_mu = 10 * np.ones(3)
    #     #random_walk_gps_ = np.sqrt(dt) * np.random.normal(0, 1, 3)
    #     #gps_bias +=  random_walk_gps_ - dt * (random_walk_mu + gps_bias)/ gps_corellation_time
    #     #affected_pos = true_pos + gps_bias + 0.5*np.array(data.offset)    
        # vel = np.array([true_pose.twist.linear.x, true_pose.twist.linear.y])
        # meas = np.concatenate((affected_pos[:2],vel))
        
        # state = kf.step(meas, dt=0.1)    
        # affected_pos[:2] = np.ravel(state[:2])
        
        if not(math.isnan(affected_pos[0]) or math.isnan(affected_pos[1]) or math.isnan(affected_pos[1])): 
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "map"
            t.child_frame_id = "affected_pos"
            t.transform.translation.x = affected_pos[0]
            t.transform.translation.y = affected_pos[1]
            t.transform.translation.z = affected_pos[2]
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
            br.sendTransform(t)
        aff_odom = Odometry()
        aff_odom.header.stamp = current_time
        aff_odom.header.frame_id = "map"
        aff_odom.child_frame_id = "hb1"
        aff_odom.pose.pose.position.x = affected_pos[0]
        aff_odom.pose.pose.position.y = affected_pos[1]
        aff_odom.pose.pose.position.z = affected_pos[2]
        #aff_odom.pose.pose.orientation = true_pose.pose.orientation
        aff_odom.twist.twist.linear.x = true_pose.twist.linear.x # Stuffing Offset and true position in the same message
        aff_odom.twist.twist.linear.y = true_pose.twist.linear.y
        aff_odom.twist.twist.linear.z = true_pose.twist.linear.z
        aff_odom.twist.twist.angular.x = true_pose.pose.position.x
        aff_odom.twist.twist.angular.y = true_pose.pose.position.y
        aff_odom.twist.twist.angular.z = true_pose.pose.position.z
        aff_odom.pose.pose.orientation.x = len(sats_vis)

        for i in range(36):
            aff_odom.pose.covariance[i] = NaN
            aff_odom.twist.covariance[i] = NaN
        odom_pub.publish(aff_odom)
        
    
def main():
    rospy.init_node('multipath_error_vis')
    br = tf2_ros.TransformBroadcaster()
    aff_odom_pub = rospy.Publisher("/multipath/hb1/aff_odom", Odometry, queue_size=10)
    get_model_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    dop_pub = rospy.Publisher('/multipath/dop', Float64MultiArray, queue_size=10)
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    sats = ['G01','G04','G07','G08', 'G09','G16','G21', 'G27']
    nav_file = data_dir + '/hksc135c.21n'
    nav_data = gr.rinexnav(nav_file, use='G')
    gps_bias = np.zeros(3)
    
    init_pose = get_model_pose("laser_0", "map")
    init_state = np.array([init_pose.pose.position.x,init_pose.pose.position.y, 0.0, 0.0, 0.0, 0.0])
    
    kf = KalmanFilter(init_state)
    offset_sub = rospy.Subscriber("/multipath/offset",
                 MultipathOffset, 
                 callback=cb_fn, 
                 callback_args=[get_model_pose, br, aff_odom_pub,
                 nav_data, dop_pub, gps_bias, kf])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__=="__main__":
    main()
