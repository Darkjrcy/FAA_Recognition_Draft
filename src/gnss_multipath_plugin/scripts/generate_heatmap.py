#!/usr/bin/env python3
import argparse
import json
import math
import os
import rclpy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from rclpy.node import Node
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from gnss_multipath_plugin.msg import GNSSMultipathFix
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import sys

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
        self.true_pos_list =[]
        self.gps_pos_list = []
        self.vis_sat_count = []
        self.dop =[]
    
    def gnss_mulitpath_fix_callback(self, data):    
        self.true_pos_list.append(data.enu_true)
        self.gps_pos_list.append(data.enu_gnss_fix)
        self.vis_sat_count.append(data.visible_sat_count) 
        self.dop.append(data.dop)
    def log_pos_data(self, data_dir, fname):
        with open(data_dir + fname + ".json", "w") as fs:
            true_pos = np.array(self.true_pos_list)
            gps_pos = np.array(self.gps_pos_list)
            vis_sat_count = np.array(self.vis_sat_count)
            dop = np.array(self.dop)
            data = {"true_pos": true_pos.tolist(),
                "gps_pos": gps_pos.tolist(),
                "vis_sat": vis_sat_count.tolist(),
                "dop": dop.tolist()
                }
            json.dump(data, fs)

    def plot_heatmap(self, data_dir, fname):
        gzb_gps_info = {}
        with open(data_dir + fname + ".json", "r") as fs:
            gzb_gps_info = json.load(fs)
        gzb_true_pos = np.array(gzb_gps_info['true_pos'])
        gzb_gps_pos = np.array(gzb_gps_info['gps_pos'])
        vis_sat = np.array(gzb_gps_info['vis_sat'])
        dop = np.array(gzb_gps_info['dop'])
        #Calculating the size of the heat map based on the lat lon values
        scale = 5
        size_x_min = (int)((np.abs(gzb_true_pos[:,0].min()))/scale)
        size_y_min = (int)((np.abs(gzb_true_pos[:,1].min()))/scale)
        size_x = (int)((np.abs(gzb_true_pos[:,0].max()))/scale + size_x_min)
        size_y = (int)((np.abs(gzb_true_pos[:,1].max()))/scale + size_y_min)
        heat_map = np.zeros((size_y+1,size_x+1))
        sat_map = np.zeros((size_y+1,size_x+1))
        dop_map = np.zeros((size_y+1,size_x+1))

        for i in range(gzb_true_pos.shape[0]):
            y_grid = (int)(gzb_true_pos[i,1]/scale + size_y_min)
            x_grid = (int)(gzb_true_pos[i,0]/scale + size_x_min)
            error = np.linalg.norm(gzb_true_pos[i,:] - gzb_gps_pos[i,:])
            if not math.isnan(error):
                if error > 30:
                    error = 30
                heat_map[y_grid,x_grid] = error
                sat_map[y_grid,x_grid] = vis_sat[i]
                dop_map[y_grid,x_grid] = dop[i,3]
            else:
                heat_map[y_grid,x_grid] = 40
                sat_map[y_grid,x_grid] = 0
                dop_map[y_grid,x_grid] = 40
        cmap1 = colors.LinearSegmentedColormap.from_list("", ["green","yellow","orange","red"])
        cmap2 = colors.LinearSegmentedColormap.from_list("", ["darkblue","blue","lightblue","white"])

        map_ = [heat_map, sat_map, dop_map]
        fig, axs = plt.subplots(1, 3, figsize=(20, 5))
        cmaps = [cmap1, cmap2, cmap1]
        clim = [[0,40],[0,8],[0,40]]
        labels = ['GNSS Error Map', 'Num of Visible Satellite', 'HDOP']
        for i in range(3):
            ax = axs[i]
            pcm = ax.pcolormesh(map_[i],
                                cmap=cmaps[i],vmin=clim[i][0], vmax=clim[i][1])
            ax.axis('off')
            fig.colorbar(pcm, ax=ax)
            ax.set_title(labels[i])
            ax.invert_yaxis()
        file_name  = data_dir + fname + '.png'    
        fig.savefig(file_name, dpi=500, bbox_inches='tight')    

class TargetTrajNode(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name=None, traj_data = None) -> None:
        super().__init__(f"{target_name}_trajectory")
        self.client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not availabel, waiting...')
        
        self.target_name = target_name
        self.traj_data = traj_data
        timer_period = 0.005
        self.timer = self.create_timer(timer_period, self.traj_callback)

        self.get_logger().info(f"setting state for {self.target_name}")
        self.state_msg = EntityState()
        self.state_msg.name = self.target_name

        self.state_msg.pose.position.x = self.traj_data[0,0]
        self.state_msg.pose.position.y = self.traj_data[0,1]
        self.state_msg.pose.position.z = 1.0
    
        self.get_logger().info(f"set initial states for {self.target_name}")
        self.request = SetEntityState.Request()
        self.index = 0
    
  
    
    def traj_callback(self):
        self.state_msg.pose.position.x = self.traj_data[self.index,0]
        self.state_msg.pose.position.y = self.traj_data[self.index,1]
        self.state_msg.pose.position.z = 1.0
        if (self.index < self.traj_data.shape[0]-1):    
            self.index +=1
        self.request.state = self.state_msg
        future = self.client.call_async(self.request)
        
        if future.done():
            response = future.result()



def main(args=None):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    start = -310
    end = 310
    num_points = int(abs(end-start +1)/4)
    car_data_x = np.linspace(start, end, num_points)
    car_data_y = np.linspace(start, end, num_points)
    car_data_gzb = np.zeros((num_points*num_points, 3))
    count = 0
    for i in range(num_points):
        for j in range(num_points):
            car_data_gzb[count, 0] = car_data_x[i]
            car_data_gzb[count, 1] = car_data_y[j]
            count+=1          
    
    gnss_data_logging_node = GNSSDataLogging()
    executor.add_node(gnss_data_logging_node)

    traj = TargetTrajNode("laser_0", car_data_gzb)
    executor.add_node(traj)    

   
    try:
        executor.spin()
    except KeyboardInterrupt:
        gnss_data_logging_node.log_pos_data(data_dir, args.fname)
        gnss_data_logging_node.plot_heatmap(data_dir, args.fname)
        gnss_data_logging_node.get_logger().info('Logging complete!')
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        gnss_data_logging_node.destroy_node()
        traj.destroy_node()
        # Shutdown the ROS client library for Python
        rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate HeatMap')

    ## General Arguments
    parser.add_argument('--fname', type=str, default='pos_data_log_heatmap',
                        help='name of the file')
    args = parser.parse_args()
    main(args)
