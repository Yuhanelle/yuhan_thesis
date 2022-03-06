#!/usr/bin/env python

import rospy
import time
import roslaunch

from nav_msgs.srv import LoadMap, LoadMapRequest

rospy.wait_for_service('/jackal1/change_map')
change_map_1 = rospy.ServiceProxy('/jackal1/change_map', LoadMap)
change_map_2 = rospy.ServiceProxy('/jackal2/change_map', LoadMap)


request1 = LoadMapRequest()
request2 = LoadMapRequest()
request1.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal_new_map.yaml'
request2.map_url = '/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/jackal_new_map.yaml'




if __name__ == '__main__':

    start_time = time.time()
    save_map = True

    while not rospy.is_shutdown():
        if time.time()-start_time > 110 and save_map==True:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/yuhan/catkin_ws/src/rnn_sim/launch/map_saver.launch'])
            launch.start()
            save_map = False
            print(time.time())
            time.sleep(1)
            result1 = change_map_1(request1)
            result2 = change_map_2(request2)
            time.sleep(5)
            rospy.signal_shutdown("map exchange done")
            
