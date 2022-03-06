#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header
import time

header = Header()
info = MapMetaData()
new_map = OccupancyGrid()
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header)
    # print("subscribe")
    
    header = data.header
    header.frame_id = rospy.get_param("~frame_id")
    header.stamp = rospy.Time.now()
    info = data.info
    new_map.data = data.data
    new_map.header = header
    new_map.info = info



def listener():

    rospy.init_node('updated_map', anonymous=True)

    rospy.Subscriber("/map", OccupancyGrid, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

def publisher():
    map_pub = rospy.Publisher('my_map', OccupancyGrid, queue_size=1)
    map_pub.publish(new_map)
    # rospy.loginfo(rospy.get_caller_id() + "I published %s", new_map.header)
    print("publishing")
    return



if __name__ == '__main__':

    start_time = time.time()

    while not rospy.is_shutdown():
        if time.time()-start_time > 10:
            listener()
            publisher()
            print(time.time())
            if time.time()-start_time > 37:
                rospy.signal_shutdown("map exchange done")
