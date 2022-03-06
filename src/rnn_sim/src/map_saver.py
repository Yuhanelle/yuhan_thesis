#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


def odom_callback(data):
    global x,y,pose,ebot_theta
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    ebot_theta=euler_from_quaternion([x,y,z,w])[2]
#def laser_callback(msg):
    #global regions
    #regions = {
     #   'bright':      ,
      #  'fright':  ,
       # 'front':   ,
       # 'fleft':   ,
        #'bleft':       ,
    #}
def Waypoints(t):
    if t == 0:
        h = 0.74
        k = 0.488
    elif t == 1:
        h = 1.42
        k = 1.289   
    elif t == 2:
        h = 1.911
        k = 1.54
    elif t == 3:
        h = 2.45
        k = 1.2
    elif t == 4:
        h = 3.141 
        k = 0 
    elif t == 5:
        h = 3.91 
        k = -1.289
    elif t == 6:
        h = 4.373
        k = -1.54 
    elif t == 7:
        h = 5.02
        k = -1.125
    elif t == 8:
        h = 5.72
        k = -0.297
    elif t == 9:
        h = 6.283
        k = 0 
    else:
        pass
  
    return [h,k]  


def main():
    rospy.init_node('ebot_controller',anonymous=True)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    i=0
    while not rospy.is_shutdown() and i<10:
        
        [x1,y1]=[x,y]
        [x2,y2]=Waypoints(i)
    
        theta_goal= math.atan((y2-y1)/(x2-x1))
        e_theta= ebot_theta-theta_goal
        velocity_msg.linear.x = 10
        velocity_msg.angular.z = (-1)*e_theta
        pub.publish(velocity_msg)
        i=i+1
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass