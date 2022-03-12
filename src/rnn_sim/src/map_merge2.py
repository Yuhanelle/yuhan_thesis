#!/usr/bin/python2.7
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import copy
import numpy as np
import time
import tf
import torch

map1 = None
map2 = None
# processing = False
merge_map = None
merged_map_publisher = None
merge_topic = "map"

def merge():
    global map1, map2, merge_map, merged_map_publisher, merge_topic
    t= time.time()
    # print(self.map1.data)
    if map1 == None or map2 == None:
        return
    grid1 = torch.as_tensor(map1.data)
    grid2 = torch.as_tensor(map2.data)
    if merge_map == None:
        print("merge map empty")
        print(map1.info)
        merge_map = OccupancyGrid()
        merge_map.header.frame_id = merge_topic
        merge_map.header.stamp = rospy.Time(0)
        merge_map.info = map1.info
        print(merge_map.header)
        # print(grid)
        grid = torch.clone(grid1)
        # check2_grid = np.subtract(grid2, grid)
        # check2_idx = np.asarray(np.where(check2_grid>0))
        # for i in check2_idx[0]:
        #     if grid2[i] == 100:
        #         grid[i] = grid[i] = np.int8(100)
        #     elif grid2[i] == 0 and grid1[i]!=100:
        #         grid[i] = np.int8(0)
        # check1_grid = np.subtract(grid1, grid)
        # check1_idx = np.asarray(np.where(check1_grid>0))
        # for i in check1_idx[0]:
        #     if grid1[i] == 100:
        #         grid[i] = grid[i] = np.int8(100)
        #     elif grid1[i] == 0 and grid1[i]!=100:
        #         grid[i] = np.int8(0)
    else:
        merge_map.header.frame_id = merge_topic
        merge_map.header.stamp = rospy.Time.from_sec(time.time())#rospy.Time(0)
        merge_map.info = map1.info
        # print(merge_map.header)
        # print(merge_map.info)
        grid = torch.as_tensor(merge_map.data)
        # check2_grid = np.subtract(grid2, grid)
        # check2_idx = np.asarray(np.where(check2_grid>0))
        # for i in check2_idx[0]:
        #     if grid2[i] == 100:
        #         grid[i] = grid[i] = np.int8(100)
        #     elif grid2[i] == 0 and grid1[i]!=100:
        #         grid[i] = np.int8(0)
        # check1_grid = np.subtract(grid1, grid)
        # check1_idx = np.asarray(np.where(check1_grid>0))
        # for i in check1_idx[0]:
        #     if grid1[i] == 100:
        #         grid[i] = grid[i] = np.int8(100)
        #     elif grid1[i] == 0 and grid1[i]!=100:
        #         grid[i] = np.int8(0)
    grid = torch.where(grid1>grid2, grid1, grid2)
    print("time before the loop", time.time()-t)

    # check_idx = np.asarray(np.where(grid2>=0))
    # # print("check_idx", check_idx[0], check_idx.shape)
    # for i in check_idx[0]:
    #     # print("i",i)
    #     # print("grid2", grid2[i])
    #     if grid2[i] == 100:
    #         grid[i] = np.int8(100)
    #     elif grid2[i] == 0 and grid1[i]!=100:
    #         grid[i] = np.int8(0)
    merge_map.data = tuple(grid)
    # print("merge map !!!!-----------------------------------------")
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
    # merge()
    return

def new_map_callback2(grid_map):
    global map2
    # print("jackal2/map header", grid_map.header)
    # print("jackal2/map info", grid_map.info)
    # print("jackal2/map data", len(grid_map.data))
    map2 = grid_map
    # merge()
    return

def main():
    global merged_map_publisher, merge_topic
    rospy.init_node("map_merge_test", anonymous=True)
    topic1 = "/jackal1/jackal1_proj_map"#rospy.get_param("~map_topic1")
    topic2 = "/jackal2/jackal2_proj_map"#rospy.get_param("~map_topic2")
    merge_topic = "/merge_map_test"#rospy.get_param("~merge_map_topic")

    
    try:
        while not rospy.is_shutdown():
            merged_map_publisher = rospy.Publisher(merge_topic, OccupancyGrid, queue_size=1)

            map1_subscriber = rospy.Subscriber(topic1, OccupancyGrid, new_map_callback1)
            map2_subscriber = rospy.Subscriber(topic2, OccupancyGrid, new_map_callback2)
            merge()
            r = rospy.Rate(5)
    except KeyboardInterrupt:
        print("Shutting down")
        
        



if __name__ == '__main__':
    main()