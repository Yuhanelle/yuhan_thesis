#!/usr/bin/python2.7
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import copy
import numpy as np
import time
import tf

# map1 = None
# map2 = None
# map3 = None
# map4 = None
# map5 = None
# map6 = None
maps = [None, None, None, None, None, None]
# processing = False
merge_map = None
merged_map_publisher = None
merge_topic = "map"
num_robot = 2

def merge():
    global maps, merge_map, merged_map_publisher, merge_topic, num_robot
    t= time.time()
    # print(self.map1.data)
    if maps[0] == None or maps[1] == None:
        return
    grid1 = np.asarray(maps[0].data)
    grid2 = np.asarray(maps[1].data)
    if merge_map == None:
        print("merge map empty")
        print(maps[0].info)
        merge_map = OccupancyGrid()
        merge_map.header.frame_id = merge_topic
        merge_map.header.stamp = rospy.Time(0)
        merge_map.info = maps[0].info
        print(merge_map.header)
        # print(grid)
        grid = np.asarray(copy.copy(grid1))
        check2_grid = np.subtract(grid2, grid)
        check2_idx = np.asarray(np.where(check2_grid>0))
        for i in check2_idx[0]:
            if grid2[i] == 100:
                grid[i] = np.int8(100)
            elif grid2[i] == 0 and grid1[i]!=100:
                grid[i] = np.int8(0)
        if num_robot >2:
            if maps[2] == None:
                return
            grid3 = np.asarray(maps[2].data)
            check3_grid = np.subtract(grid3, grid)
            check3_idx = np.asarray(np.where(check3_grid>0))
            for i in check3_idx[0]:
                if grid3[i] == 100:
                    grid[i] = np.int8(100)
                elif grid3[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)
        if num_robot >3:
            if maps[3] == None:
                return
            grid4 = np.asarray(maps[3].data)
            check4_grid = np.subtract(grid4, grid)
            check4_idx = np.asarray(np.where(check4_grid>0))
            for i in check4_idx[0]:
                if grid4[i] == 100:
                    grid[i] = np.int8(100)
                elif grid4[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)
        if num_robot > 4:
            if maps[4] == None:
                return
            grid5 = np.asarray(maps[4].data)
            check5_grid = np.subtract(grid5, grid)
            check5_idx = np.asarray(np.where(check5_grid>0))
            for i in check5_idx[0]:
                if grid5[i] == 100:
                    grid[i] = np.int8(100)
                elif grid5[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)
        if num_robot >5:
            if maps[5] == None:
                return
            grid6 = np.asarray(maps[5].data)
            check6_grid = np.subtract(grid6, grid)
            check6_idx = np.asarray(np.where(check6_grid>0))
            for i in check6_idx[0]:
                if grid6[i] == 100:
                    grid[i] = np.int8(100)
                elif grid6[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)

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
        merge_map.info = maps[0].info
        # print(merge_map.header)
        # print(merge_map.info)
        grid = np.asarray(merge_map.data)
        check2_grid = np.subtract(grid2, grid)
        check2_idx = np.asarray(np.where(check2_grid>0))
        for i in check2_idx[0]:
            if grid2[i] == 100:
                grid[i] = np.int8(100)
            elif grid2[i] == 0 and grid1[i]!=100:
                grid[i] = np.int8(0)
        check1_grid = np.subtract(grid1, grid)
        check1_idx = np.asarray(np.where(check1_grid>0))
        for i in check1_idx[0]:
            if grid1[i] == 100:
                grid[i] = np.int8(100)
            elif grid1[i] == 0 and grid1[i]!=100:
                grid[i] = np.int8(0)

        if num_robot >2:
            if maps[2] == None:
                return
            grid3 = np.asarray(maps[2].data)
            check3_grid = np.subtract(grid3, grid)
            check3_idx = np.asarray(np.where(check3_grid>0))
            for i in check3_idx[0]:
                if grid3[i] == 100:
                    grid[i] = np.int8(100)
                elif grid3[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)
        if num_robot >3:
            if maps[3] == None:
                return
            grid4 = np.asarray(maps[3].data)
            check4_grid = np.subtract(grid4, grid)
            check4_idx = np.asarray(np.where(check4_grid>0))
            for i in check4_idx[0]:
                if grid4[i] == 100:
                    grid[i] = np.int8(100)
                elif grid4[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)
        if num_robot > 4:
            if maps[4] == None:
                return
            grid5 = np.asarray(maps[4].data)
            check5_grid = np.subtract(grid5, grid)
            check5_idx = np.asarray(np.where(check5_grid>0))
            for i in check5_idx[0]:
                if grid5[i] == 100:
                    grid[i] = np.int8(100)
                elif grid5[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)
        if num_robot >5:
            if maps[5] == None:
                return
            grid6 = np.asarray(maps[5].data)
            check6_grid = np.subtract(grid6, grid)
            check6_idx = np.asarray(np.where(check6_grid>0))
            for i in check6_idx[0]:
                if grid6[i] == 100:
                    grid[i] = np.int8(100)
                elif grid6[i] == 0 and grid[i]!=100:
                    grid[i] = np.int8(0)

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
    global maps
    # print("jackal1/map header", grid_map.header)
    # print("jackal1/map info", grid_map.info)
    # print("jackal1/map data", len(grid_map.data))
    # rospy.loginfo("New j1 map was set")
    maps[0] = grid_map
    # merge()
    return

def new_map_callback2(grid_map):
    global maps
    # print("jackal2/map header", grid_map.header)
    # print("jackal2/map info", grid_map.info)
    # print("jackal2/map data", len(grid_map.data))
    maps[1] = grid_map
    # merge()
    return

def new_map_callback3(grid_map):
    global maps
    # rospy.loginfo("New j3 map was set")
    maps[2] = grid_map
    # print("jackal3/map header", map3.header)
    # print("jackal3/map info", map3.info)
    # print("jackal3/map data", len(map3.data))
    # merge()
    return

def new_map_callback4(grid_map):
    global maps
    # print("jackal4/map header", grid_map.header)
    # print("jackal4/map info", grid_map.info)
    # print("jackal4/map data", len(grid_map.data))
    maps[3] = grid_map
    # merge()
    return
    
def new_map_callback5(grid_map):
    global maps
    # print("jackal5/map header", grid_map.header)
    # print("jackal5/map info", grid_map.info)
    # print("jackal5/map data", len(grid_map.data))
    # rospy.loginfo("New j5 map was set")
    maps[4] = grid_map
    # merge()
    return

def new_map_callback6(grid_map):
    global maps
    # rospy.loginfo("New j6 map was set")
    maps[5] = grid_map
    # print("jackal6/map header", map6.header)
    # print("jackal6/map info", map6.info)
    # print("jackal6/map data", len(map6.data))
    # merge()
    return

def main():
    global merged_map_publisher, merge_topic, num_robot
    rospy.init_node("map_merge_test", anonymous=True)
    # topic1 = rospy.get_param("~map_topic1")
    # topic2 = rospy.get_param("~map_topic2")
    topic = rospy.get_param("~map_topics")
    num_robot = rospy.get_param("~num_robot")
    # merge_topic = rospy.get_param("~merge_map_topic")

    
    try:
        while not rospy.is_shutdown():
            merged_map_publisher = rospy.Publisher(merge_topic, OccupancyGrid, queue_size=1)

            map1_subscriber = rospy.Subscriber("/jackal1/"+topic, OccupancyGrid, new_map_callback1)
            map2_subscriber = rospy.Subscriber("/jackal2/"+topic, OccupancyGrid, new_map_callback2)
            if num_robot>2:
                map3_subscriber = rospy.Subscriber("/jackal3/"+topic, OccupancyGrid, new_map_callback3)
            if num_robot>3:
                map4_subscriber = rospy.Subscriber("/jackal4/"+topic, OccupancyGrid, new_map_callback4)
            if num_robot>4:
                map5_subscriber = rospy.Subscriber("/jackal5/"+topic, OccupancyGrid, new_map_callback5)
            if num_robot>5:
                map6_subscriber = rospy.Subscriber("/jackal6/"+topic, OccupancyGrid, new_map_callback6)
            merge()
            r = rospy.Rate(5)
    except KeyboardInterrupt:
        print("Shutting down")
        
        



if __name__ == '__main__':
    main()