# import yaml
import pickle
import numpy as np
from math import *
import matplotlib.pyplot as plt
# import sys, rospy, roslaunch, os
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from robot_localization.srv import SetPose
import time

# TO RUN: python launch_gazebo.py TL S 3 30
# - for Top Left corner spawn, facing South, team size of 3, in a 30x30 env

def get_gw_coord(corner, team_size, env_size):
    ''' 
    - get grid world coordinates for all possible robot starting positions 
    - assumes robots start in corners, with team sizes between 2-6
    '''
    gw_coord = []
    init_pos = [[(0,0),(0,1),(1,0), (1,1), (0,2), (2,0)], 
                [(0,env_size-1),(0,env_size-2),(1,env_size-1), (1,env_size-2), (0, env_size-3), (2,env_size-1)], 
                [(env_size-1,env_size-1),(env_size-2,env_size-1),(env_size-1,env_size-2), (env_size-2,env_size-2), (env_size-3, env_size-1), (env_size-1,env_size-3)], 
                [(env_size-1,0),(env_size-2,0),(env_size-1,1), (env_size-2,1), (env_size-3,0), (env_size-1,2)]]
    
    if corner == 'TL':
        gw_coord = init_pos[0][:team_size]
    elif corner == 'TR':
        gw_coord = init_pos[1][:team_size]
    elif corner == 'BR':
        gw_coord = init_pos[2][:team_size]
    elif corner == 'BL':
        gw_coord = init_pos[3][:team_size]
    
    return gw_coord

def transform_pt(gw_pt, env_size):
    '''
    transform point from gridworld to gazebo coordinates
    '''
    tran_matrix = [-(env_size-1)/2, (env_size-1)/2]
    
    # define transformation matrices
    rotation = np.array([[0, 1],[-1,0]])
    translation = np.array(tran_matrix) # adjust based on grid size
    
    # convert from one frame to another
    gz_pt = np.matmul(rotation, gw_pt) + translation
    
    return tuple(gz_pt.reshape(1, -1)[0])

def gw_to_gz(gw_pts, env_size):
    gz_coord = []
    for pt in gw_pts:
        gz_coord.append(transform_pt(pt, env_size))
    return gz_coord

def create_config(team_size, direction, init_pos):
    ''' Generate configuration to set robot init pose in launch file '''
    
    if direction == 'N':
        rad = 1.57
    elif direction == 'E':
        rad = 0
    elif direction == 'S':
        rad = 4.71
    elif direction == 'W':
        rad = 3.14
    
    data = {}
    
    for idx, rbt in enumerate(range(team_size)):
        x_pos = 'j'+str(idx+1)+'_x_pos'
        y_pos = 'j'+str(idx+1)+'_y_pos'
        z_pos = 'j'+str(idx+1)+'_z_pos'
        yaw = 'j'+str(idx+1)+'_yaw'
        
        data[x_pos] = float(init_pos[idx][0])
        data[y_pos] = float(init_pos[idx][1])
        data[z_pos] = 0
        data[yaw] = rad
    
    return data

def calibrate_pt(pt, direction):
    ''' Pre-programed calibration pose based on the direction '''
    x = pt[0]
    y = pt[1]
    
    if direction == 'N':
        calibrated_pt = (y, -x)
    elif direction == 'E':
        calibrated_pt = (x, y)
    elif direction == 'S':
        calibrated_pt = (-y, x)
    elif direction == 'W':
        calibrated_pt = (-x, -y)

    return calibrated_pt

def call_setpose(pose_msg, service_name):
    ''' call ros service '''
    rospy.wait_for_service(service_name)
    try:
        set_pose = rospy.ServiceProxy(service_name, SetPose)
        resp1 = set_pose(pose_msg)
    except rospy.ServiceException as exc:
        print("Service call failed")

def calibration_service():
    ''' Calibrate the pose of each robot '''
    for rbt_idx, coord in enumerate(calibrated_coord):
        
        service_name = 'jackal'+str(rbt_idx+1)+"/set_pose"
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose.position.x = coord[0]
        pose_msg.pose.pose.position.y = coord[1]
        pose_msg.pose.pose.orientation.w = 1
        pose_msg.header.frame_id = 'jackal'+str(rbt_idx+1)+"/base_link"
        
        call_setpose(pose_msg, service_name)

if __name__ == "__main__":


    # # take user defined input
    # if len(sys.argv) == 5:
    #     corner = str(sys.argv[1])
    #     direction = str(sys.argv[2])
    #     team_size = int(sys.argv[3])
    #     env_size = int(sys.argv[4])
    # else:
    #     print("Not enough arguments")

    corner = 'TL'
    direction = 'S'
    team_size = 6
    env_size = 30

    # convert from gridworld coordinates to gazebo coordinates
    gw_coord = get_gw_coord(corner, team_size, env_size)
    gz_coord = gw_to_gz(gw_coord,env_size)

    print(gz_coord)



    # # define configuration
    # data = create_config(team_size, direction, gz_coord)
    # config = ''
    # for param in list(data.keys()):
    #     config+=' '+param+':='+str(data[param])

    # # whether to spawn a robot based on team size    
    # for rbt in range(6):
    #     if rbt < team_size:
    #         config+=' '+'j'+str(rbt+1)+'_spawn:=true'
    #     else:
    #         config+=' '+'j'+str(rbt+1)+'_spawn:=false'

    # # set world name config
    # config+=' world_name:=/home/asblab/yuhan/catkin_ws/src/jackal_simulator/jackal_gazebo_rtab/worlds/jackal_'+str(env_size)+'_2.world'

    # # run roslaunch command with config
    # os.system("roslaunch jackal_gazebo_rtab jackal_rtab_aaron.launch"+config)
    # # # wait a bit ..
    # # time.sleep(5)

    # # print("HEREEEEE")

    # # # run rosservice calls for rviz
    # # calibrated_coord = []
    # # for pt in gz_coord:
    # #     calibrated_coord.append(calibrate_pt(pt, direction))

    # # # call the service
    # # calibration_service()
