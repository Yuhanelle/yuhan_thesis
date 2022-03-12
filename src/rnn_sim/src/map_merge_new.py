#!/usr/bin/python2.7
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import copy
import numpy as np
import time
import tf

map1 = None
map2 = None
map3 = None
map4 = None
map5 = None
map6 = None

# processing = False
merge_map = None
merged_map_publisher = None
merge_topic = "map"
map_topics=None
num_robot = 2

def merge():
    global map1, map2, map3, map4, map5, map6
    global merge_map, merged_map_publisher, merge_topic
    t= time.time()
    if map1 == None or map2 == None :
        # print("map1 and map2 empty")
        # print("check merge", map1.header, m2.header)
        return
    # print(map1.header)
    grid1 = np.asarray(map1.data)
    grid2 = np.asarray(map2.data)
    if merge_map == None:
        merge_map = OccupancyGrid()
    merge_map.header.frame_id = merge_topic
    merge_map.header.stamp = rospy.Time.from_sec(time.time())
    merge_map.info = map1.info
    # print(merge_map.header)
    grid = np.maximum(grid1, grid2)
    if num_robot ==3:
        if map3 == None:
            print("map3 empty")
            return
        grid3 = np.asarray(map3.data)
        grid = np.maximum(grid, grid3)
    elif num_robot ==4:
        if map3 == None or map4 == None:
            return
        grid3 = np.asarray(map3.data)
        grid4 = np.asarray(map4.data)
        grid_temp = np.maximum(grid3, grid4)
        grid = np.maximum(grid, grid_temp)
    elif num_robot==5:
        if map3 == None or map4 == None or map5 ==None:
            return
        grid3 = np.asarray(map3.data)
        grid4 = np.asarray(map4.data)
        grid5 = np.asarray(map5.data)
        grid_temp = np.maximum(grid3, grid4)
        grid = np.maximum(grid, grid_temp)
        grid = np.maximum(grid, grid5)
    elif num_robot==6:
        if map3 == None or map4 == None or map5 == None or map6 == None:
            return
        grid3 = np.asarray(map3.data)
        grid4 = np.asarray(map4.data)
        grid5 = np.asarray(map5.data)
        grid5 = np.asarray(map6.data)
        grid_temp1 = np.maximum(grid3, grid4)
        grid_temp2 = np.maximum(grid5, grid6)
        grid_temp = np.maximum(grid_temp1, grid_temp2)
        grid = np.maximum(grid, grid_temp)
        grid = np.maximum(grid, grid5)
    merge_map.data = tuple(grid)
    print("merge map !!!!-----------------------------------------")
    merged_map_publisher.publish(merge_map)
    print("time used to merge the map", time.time()-t)
    return

def new_map_callback1(grid_map):
    global map1
    # print("jackal1/map header", grid_map.header)
    # print("jackal1/map info", grid_map.info)
    # print("jackal1/map data", len(grid_map.data))
    # rospy.loginfo("New j1 map was set")
    map1 = grid_map
    print("jackal1/map header", map1.header)
    print("jackal1/map info", map1.info)
    print("jackal1/map data", len(map1.data))
    # merge()
    return

def new_map_callback2(grid_map):
    global map2
    map2 = grid_map
    # print("jackal2/map header", map2.header)
    # print("jackal2/map info", map2.info)
    # print("jackal2/map data", len(map2.data))
    # merge()
    return
def new_map_callback3(grid_map):
    global map3
    # rospy.loginfo("New j1 map was set")
    map3 = grid_map
    print("jackal3/map header", map3.header)
    print("jackal3/map info", map3.info)
    print("jackal3/map data", len(map3.data))
    # merge()
    return

def new_map_callback4(grid_map):
    global map4
    # print("jackal2/map header", grid_map.header)
    # print("jackal2/map info", grid_map.info)
    # print("jackal2/map data", len(grid_map.data))
    map4 = grid_map
    # merge()
    return
    
def new_map_callback5(grid_map):
    global map5
    # print("jackal1/map header", grid_map.header)
    # print("jackal1/map info", grid_map.info)
    # print("jackal1/map data", len(grid_map.data))
    # rospy.loginfo("New j1 map was set")
    map5 = grid_map
    # merge()
    return

def new_map_callback6(grid_map):
    global map6
    # rospy.loginfo("New j6 map was set")
    map6 = grid_map
    # print("jackal2/map header", map6.header)
    # print("jackal2/map info", map6.info)
    # print("jackal2/map data", len(map6.data))
    # merge()
    return
def main():
    global merged_map_publisher, merge_topic, num_robot, topic
    rospy.init_node("map_merge_test", anonymous=True)
    topic = rospy.get_param("~map_topics")
    num_robot = rospy.get_param("~num_robot")
    merge_topic = rospy.get_param("~merge_map_topic")

    
    try:
        while not rospy.is_shutdown():
            merged_map_publisher = rospy.Publisher(merge_topic, OccupancyGrid, queue_size=1)

            map1_subscriber = rospy.Subscriber("/jackal1/"+topic, OccupancyGrid, new_map_callback1)
            map2_subscriber = rospy.Subscriber("/jackal2/"+topic, OccupancyGrid, new_map_callback2)
            if num_robot >2:
                map3_subscriber = rospy.Subscriber("/jackal3/"+topic, OccupancyGrid, new_map_callback3)
            if num_robot >3:
                map4_subscriber = rospy.Subscriber("/jackal4/"+topic, OccupancyGrid, new_map_callback4)
            if num_robot >4:
                map5_subscriber = rospy.Subscriber("/jackal5/"+topic, OccupancyGrid, new_map_callback5)
            if num_robot == 6:
                map6_subscriber = rospy.Subscriber("/jackal6/"+topic, OccupancyGrid, new_map_callback6)
            merge()
            r = rospy.Rate(5)
    except KeyboardInterrupt:
        print("Shutting down")
        
        



if __name__ == '__main__':
    main()