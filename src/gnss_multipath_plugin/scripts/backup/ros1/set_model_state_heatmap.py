#!/usr/bin/env python3
import os
import numpy as np
from numpy.core.arrayprint import printoptions
from numpy.linalg import pinv

import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import geometry_msgs.msg
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import json
from tf.transformations import quaternion_from_euler


def target_traj_gps(t, *args):
    car_data_gzb = args[1]
    index = int(t*10 % car_data_gzb.shape[0])
    pos = [car_data_gzb[index,0], car_data_gzb[index,1], 1]
    att = np.array([0, 0, car_data_gzb[index, 2]])
    return np.concatenate([pos, att])

def set_drone_state(*args):
    args = args[0]
    model_name = args[0]
    traj_fn = args[1]
    traj_fn_args = args[2]
    begin = traj_fn_args[0]

    state_msg_0 = ModelState()
    state_msg_0.model_name = model_name
    state_msg_0.pose.position.x = begin[0]
    state_msg_0.pose.position.y = begin[1]
    state_msg_0.pose.position.z = begin[2]
    state_msg_0.pose.orientation.x = 0
    state_msg_0.pose.orientation.y = 0
    state_msg_0.pose.orientation.z = 0
    state_msg_0.pose.orientation.w = 1

    rospy.wait_for_service('/gazebo/set_model_state')
    start = rospy.get_rostime()
    elapsed = 0
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        elapsed = (now - start).to_sec()
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            pose = traj_fn(elapsed, *traj_fn_args)
            state_msg_0.pose.position.x = pose[0]
            state_msg_0.pose.position.y = pose[1]
            state_msg_0.pose.position.z = pose[2]
            
            q_ = quaternion_from_euler(pose[3], pose[4], pose[5])
            state_msg_0.pose.orientation.x = q_[0]
            state_msg_0.pose.orientation.y = q_[1]
            state_msg_0.pose.orientation.z = q_[2]
            state_msg_0.pose.orientation.w = q_[3]
            
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg_0 )

        except rospy.ServiceException as e:
            print("Service call failed: {:s}".format(str(e)))
        rate.sleep()



def main():
    rospy.init_node('set_pose')
    start = -500
    end = 500
    num_points = int(abs(end-start +1)/10)
    car_data_x = np.linspace(start,end, num_points)
    car_data_y = np.linspace(start,end,num_points)
    car_data_gzb = np.zeros((num_points*num_points, 3))
    count = 0
    for i in range(num_points):
        for j in range(num_points):
            car_data_gzb[count, 0] = car_data_x[i]
            car_data_gzb[count, 1] = car_data_y[j]
            count+=1          
    x0_car = np.array([car_data_gzb[0,0], car_data_gzb[0,1], 1])

    # each executor_arg corresponds to a model in gazebo
    # each executor_arg should have 3 items [model_name, traj_fn_name, traj_fn_args]
    executor_args = [
        ["laser_0", target_traj_gps, [x0_car, car_data_gzb]],
    ]
    
    with ThreadPoolExecutor(max_workers=5) as tpe:
       tpe.map(set_drone_state, executor_args)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
