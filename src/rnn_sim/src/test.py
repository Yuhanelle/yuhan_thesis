#!/usr/bin/python2.7
import rospy
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
# from visualization_msgs.msg import MarkerArray, Marker
# import visualization_msgs
import copy
import numpy as np
import time
import torch
def main():
    # a = np.zeros((20,))
    # print(a.shape)
    # b = np.where(a==0)
    # print(b[0][0])
    # print(b)
    # t = time.time()
    # a = 10
    # b = 0
    # for i in range(1621):
    #     for j in range(1621):
    #         c = a+b
    # print(time.time()-t)
    print(torch.cuda.is_available())
    print(torch.cuda.current_device())
    print(torch.cuda.device(0))
    
if __name__ == '__main__':
    main()