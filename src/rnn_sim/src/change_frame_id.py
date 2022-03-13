#!/usr/bin/python2.7

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')
odom = Odometry()
header = Header()
header.frame_id = rospy.get_param("~ground_frame_id")
child_frame_id = rospy.get_param("~child_frame_id")
from_topic_name = rospy.get_param("~from_odom_topic")
to_topic_name = rospy.get_param("~to_odom_topic")


# rospy.wait_for_service('/gazebo/get_model_state')
# get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
def callback(data):
    global header, odom
    odom = data
    odom.header = header
    header.stamp = rospy.Time.now()
    # odom.child_frame_id = child_frame_id

odom_sub = rospy.Subscriber(from_topic_name, Odometry, callback)
odom_pub = rospy.Publisher(to_topic_name, Odometry, queue_size=10)


    


r = rospy.Rate(5)

while not rospy.is_shutdown():
    
    odom_pub.publish(odom)

    r.sleep()