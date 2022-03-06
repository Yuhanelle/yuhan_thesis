#!/usr/bin/env python

import rospy
import time
import roslaunch
import csv
import math

from nav_msgs.srv import LoadMap, LoadMapRequest
from nav_msgs.msg import Odometry







# global parameter number of robots
#num_robot = rospy.get_param('~num_robot')
num_robot = 2
ground_map = []




def lineLine(x1, y1, x2, y2, x3, y3, x4, y4):
    #calculate the direction of the lines
    uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)+0.001)
    uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)+0.001)

    # if uA and uB are between 0-1, lines are colliding
    if (0<= uA <= 1 and 0<= uB <= 1):
        return True
    return False

def lineRect(x1, y1, x2, y2, rx, ry, rw, rh):
    # checck if the line has hit any of the rectangle's sides
    left = lineLine(x1,y1,x2,y2, rx,ry,rx, ry+rh)
    right = lineLine(x1,y1,x2,y2, rx+rw,ry, rx+rw,ry+rh)
    top = lineLine(x1,y1,x2,y2, rx,ry, rx+rw,ry)
    bottom = lineLine(x1,y1,x2,y2, rx,ry+rh, rx+rw,ry+rh)
    # if any of the above is true, the line has hit the rectangle
    if left or right or top or bottom:
        return True
    return False

def check_forbidden_communication(p1, p2):
    # calculate which grid cells are crossed
    offset = map_size/2
    x1 = offset - math.ceil(p1[0])
    y1 = offset - math.ceil(p1[1])
    x2 = offset - math.ceil(p2[0])
    y2 = offset - math.ceil(p2[1])
    # accumulate the surrounding cells occupied by obstacles
    surrounding_cells = []
    for i in range(int(min(x1, x2)), int(max(x1, x2))+1):
        for j in range(int(min(y1, y2)), int(max(y1, y2)+1)):
            print(j, i, ground_map[j][i])
            if ground_map[j][i] == 1:
                surrounding_cells.append([i, j])
                # add the index of cell

    # print(surrounding_cells)
    if surrounding_cells == []:
        # no obstacle
        return False
    # check for collision
    for block in surrounding_cells:
        collision = lineRect(offset-p1[0], offset-p1[1], offset-p2[0], offset-p2[1], block[0], block[1], 1, 1)
        if collision == True:
            return True
    return False


def map_merge_check():
    print(j1_position, j2_position)
    if (j1_position[0]-j2_position[0])**2 + (j1_position[1]-j2_position[1])**2 < 25:
        j1_j2_merge = True
        print("robot close enough")
        if j1_j2_merge and not check_forbidden_communication(j1_position, j2_position):
            print("j1_j2 need to merge map")
            global j1_j2_save
            j1_j2_save = True

def callback1(msg):
    pose = msg.pose.pose
    global j1_position
    j1_position = [pose.position.y, pose.position.x]
    map_merge_check()
    #print(j1_position)

def callback2(msg):
    pose = msg.pose.pose
    global j2_position
    j2_position = [pose.position.y, pose.position.x]
    map_merge_check()
    #print(j2_position)


def sub_odom():

    rospy.wait_for_message('/jackal1/ground_odom', Odometry)
    rospy.Subscriber('/'+ namespace1 +'/ground_odom', Odometry, callback1)
    rospy.Subscriber('/'+ namespace2 +'/ground_odom', Odometry, callback2)
    rospy.loginfo("Subscribing to ground_odom of robot "+namespace1 +" and "+namespace2)
    



if __name__ == '__main__':

    # encode map to forbidden inappropriate map merge actions
    global map_size
    map_size = 30
    results = []
    with open("/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/map1.csv") as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='"') # change contents to floats
        for row in reader: # each row is a list
            results.append(row)
    results[0][0] = '0'
    global ground_map
    ground_map = [[int(j) for j in i] for i in results]

    # print(check_forbidden_communication([14, 1], [12, 5.5]))
    global j1_position
    global j2_position 
    global j1_j2_save

    rospy.init_node('sub_odom', anonymous=True)
    global j1_x
    global j1_y
    global j2_x
    global j2_y
    global namespace1
    global namespace2
    global map_ns
    j1_x = rospy.get_param('~j1_x')
    j1_y = rospy.get_param('~j1_y')
    j2_x = rospy.get_param('~j2_x')
    j2_y = rospy.get_param('~j2_y')
    namespace1 = rospy.get_param('~ns1')
    namespace2 = rospy.get_param('~ns2')
    map_ns = rospy.get_param('~map_ns')

    
    rospy.wait_for_service('/'+namespace1+'/change_map')
    change_map_1 = rospy.ServiceProxy('/'+namespace1+'/change_map', LoadMap)
    change_map_2 = rospy.ServiceProxy('/'+namespace2+'/change_map', LoadMap)

    request1 = LoadMapRequest()
    request2 = LoadMapRequest()

    request1.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/'+namespace1+ '_new_map.yaml'
    request2.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/'+namespace2+ '_new_map.yaml'

    j1_j2_save = False
    j1_position = [float(j1_y), float(j1_x)]
    j2_position = [-float(j2_y), float(j2_x)]


    while not rospy.is_shutdown():
        sub_odom()
        if j1_j2_save:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            cli_args1 = ['/home/yuhan/catkin_ws/src/rnn_sim/launch/map_saver_'+namespace1+'.launch',  'map:='+map_ns+'/map']
            cli_args2 = ['/home/yuhan/catkin_ws/src/rnn_sim/launch/map_saver_'+namespace2+'.launch', 'map:='+map_ns+'/map']
            # roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(['/home/yuhan/catkin_ws/src/rnn_sim/launch/map_saver_'+namespace1+'.launch'])
            roslaunch_args1 = cli_args1[1:]
            launch_file1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0], roslaunch_args1)]
            print("save global map for map exchange jackal 1")
            roslaunch_args2 = cli_args2[1:]
            launch_file2 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0], roslaunch_args2)]
            print("save global map for map exchange jackal 2")
            

            # roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)
            # roslaunch_args2 = cli_args2[2:]

            # launch_files = [(roslaunch_file1, roslaunch_args1), (roslaunch_file2, roslaunch_args2)]

            parent1 = roslaunch.parent.ROSLaunchParent(uuid, launch_file1)
            parent1.start()
            parent2 = roslaunch.parent.ROSLaunchParent(uuid, launch_file2)
            parent2.start()

            result1 = change_map_1(request1)
            result2 = change_map_2(request2)
            time.sleep(8)
            j1_j2_save = False


        # if time.time()-start_time > 110 and save_map==True:
        #     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #     roslaunch.configure_logging(uuid)
        #     launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/yuhan/catkin_ws/src/rnn_sim/launch/map_saver.launch'])
        #     launch.start()
        #     save_map = False
        #     print(time.time())
        #     time.sleep(1)
        #     result1 = change_map_1(request1)
        #     result2 = change_map_2(request2)
        #     time.sleep(5)
        #     rospy.signal_shutdown("map exchange done")
            
