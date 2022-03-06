#!/usr/bin/env python

import rospy
import time
import roslaunch
import csv
import math

from nav_msgs.srv import LoadMap, LoadMapRequest
from nav_msgs.msg import Odometry

rospy.wait_for_service('/jackal1/change_map')
change_map_1 = rospy.ServiceProxy('/jackal1/change_map', LoadMap)
change_map_2 = rospy.ServiceProxy('/jackal2/change_map', LoadMap)
change_map_3 = rospy.ServiceProxy('/jackal3/change_map', LoadMap)
change_map_4 = rospy.ServiceProxy('/jackal4/change_map', LoadMap)
change_map_5 = rospy.ServiceProxy('/jackal5/change_map', LoadMap)
change_map_6 = rospy.ServiceProxy('/jackal6/change_map', LoadMap)


request1 = LoadMapRequest()
request2 = LoadMapRequest()
request3 = LoadMapRequest()
request4 = LoadMapRequest()
request5 = LoadMapRequest()
request6 = LoadMapRequest()
request1.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal1_new_map.yaml'
request2.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal2_new_map.yaml'
request3.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal3_new_map.yaml'
request4.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal4_new_map.yaml'
request5.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal5_new_map.yaml'
request6.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal6_new_map.yaml'





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

    # check map merging condition for jackal 1, 2, 3 (three robots case)
    if num_robot >=3:
        if (j1_position[0]-j3_position[0])**2 + (j1_position[1]-j3_position[1])**2 < 16:
            j1_j3_merge = True
        if (j2_position[0]-j3_position[0])**2 + (j2_position[1]-j3_position[1])**2 < 16:
            j2_j3_merge = True
    # check map merging condition for jackal 1,2,3,4 (four robots case)
    if num_robot >=4:
        if (j1_position[0]-j4_position[0])**2 + (j1_position[1]-j4_position[1])**2 < 16:
            j1_j4_merge = True
        if (j2_position[0]-j4_position[0])**2 + (j2_position[1]-j4_position[1])**2 < 16:
            j2_j4_merge = True
        if (j3_position[0]-j4_position[0])**2 + (j3_position[1]-j4_position[1])**2 < 16:
            j3_j4_merge = True 
    # check map merging condition for jackal 1,2,3,4,5 (five robots case)
    if num_robot >=5:
        if (j1_position[0]-j5_position[0])**2 + (j1_position[1]-j5_position[1])**2 < 16:
            j1_j5_merge = True
        if (j2_position[0]-j5_position[0])**2 + (j2_position[1]-j5_position[1])**2 < 16:
            j2_j5_merge = True
        if (j3_position[0]-j5_position[0])**2 + (j3_position[1]-j5_position[1])**2 < 16:
            j3_j5_merge = True 
        if (j4_position[0]-j5_position[0])**2 + (j4_position[1]-j5_position[1])**2 < 16:
            j4_j5_merge = True
    if num_robot == 6:
        if (j1_position[0]-j6_position[0])**2 + (j1_position[1]-j6_position[1])**2 < 16:
            j1_j6_merge = True
        if (j2_position[0]-j6_position[0])**2 + (j2_position[1]-j6_position[1])**2 < 16:
            j2_j6_merge = True
        if (j3_position[0]-j6_position[0])**2 + (j3_position[1]-j6_position[1])**2 < 16:
            j3_j6_merge = True 
        if (j4_position[0]-j6_position[0])**2 + (j4_position[1]-j6_position[1])**2 < 16:
            j4_j6_merge = True
        if (j5_position[0]-j6_position[0])**2 + (j5_position[1]-j6_position[1])**2 < 16:
            j5_j6_merge = True
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

def callback3(msg):
    pose = msg.pose.pose
    global j3_position
    j3_position = [pose.position.y, pose.position.x]

def callback4(msg):
    pose = msg.pose.pose
    global j4_position
    j4_position = [pose.position.y, pose.position.x]

def callback5(msg):
    pose = msg.pose.pose
    global j5_position
    j5_position = [pose.position.y, pose.position.x]

def callback6(msg):
    pose = msg.pose.pose
    global j6_position
    j6_position = [pose.position.y, pose.position.x]

def sub_odom():
    rospy.init_node('sub_odom', anonymous=True)
    rospy.wait_for_message('/jackal1/ground_odom', Odometry)
    for i in range(num_robot):
        # robot_name = 'jackal' + str(i+1)
        # print(robot_name)
        # subscribe to each jackal odom topics
        if i == 0:
            rospy.Subscriber('/jackal1/ground_odom', Odometry, callback1)
        elif i == 1:
            rospy.Subscriber('/jackal2/ground_odom', Odometry, callback2)
        elif i == 2:
            rospy.Subscriber('/jackal3/ground_odom', Odometry, callback3)
        elif i == 3:
            rospy.Subscriber('/jackal4/ground_odom', Odometry, callback4)
        elif i == 4:
            rospy.Subscriber('/jackal5/ground_odom', Odometry, callback5)
        elif i == 5:
            rospy.Subscriber('/jackal6/ground_odom', Odometry, callback6)
        rospy.loginfo("Subscribing to ground_odom of robot "+str(i+1))
    
    # check map merging condition for jackal 1 and 2 (two robots case)
    
    # rospy.spin()



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
    global j3_position
    global j4_position
    global j5_position
    global j6_position
    global j1_j2_save
    global j1_j3_save
    global j1_j4_save
    global j1_j5_save
    global j1_j6_save
    global j2_j3_save
    global j2_j4_save
    global j2_j5_save
    global j2_j6_save
    global j3_j4_save
    global j3_j5_save
    global j3_j6_save
    global j4_j5_save
    global j4_j6_save
    j1_j2_save = False
    j1_j3_save = False
    j1_j4_save = False
    j1_j5_save = False
    j1_j6_save = False
    j2_j3_save = False
    j2_j4_save = False
    j2_j5_save = False
    j2_j6_save = False
    j3_j4_save = False
    j3_j5_save = False
    j3_j6_save = False
    j4_j5_save = False
    j4_j6_save = False
    j1_j2_forbidden = False
    j1_position = [14, 14]
    j2_position = [-14, -14]
    while not rospy.is_shutdown():
        sub_odom()
        if j1_j2_save and ! j1_j2_fobidden:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            cli_args1 = ['/home/yuhan/catkin_ws/src/rnn_sim/launch/map_saver_1.launch',  'map:=/map']
            cli_args2 = ['rnn_sim', 'map_saver_2.launch', 'map:=/jackal1/map']
            roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(['/home/yuhan/catkin_ws/src/rnn_sim/launch/map_saver.launch'])
            roslaunch_args1 = cli_args1[1:]
            launch_files = [(roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0], roslaunch_args1)]

            # roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)
            # roslaunch_args2 = cli_args2[2:]

            # launch_files = [(roslaunch_file1, roslaunch_args1), (roslaunch_file2, roslaunch_args2)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

            parent.start()
            result1 = change_map_1(request1)
            result2 = change_map_2(request2)


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
            
