#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from navigation.msg import Pathvector

import math
 
roll = pitch = yaw = 0.0
target = 0
length=0
kp=0.5
 
def get_target (msg):
    global target,length
    x=msg.x
    y=msg.y   
    target=math.atan2(y,x)	
    length=math.sqrt(x*x+y*y)
    #print target


def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print yaw
 
rospy.init_node('pathfollow_node')
 
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
sub2= rospy.Subscriber ('/pathv', Pathvector, get_target)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10)
command =Twist()
 
while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    dif=target-yaw
    tangel=math.pi-abs(target)+math.pi-abs(yaw)
    if abs(target-yaw)>tangel:
        dif=math.copysign(tangel,-dif)

    command.angular.z = kp * dif
    if abs(dif)<0.6:
        command.linear.x=0.1
    else:
        command.linear.x=0.02
    pub.publish(command)
    print("taeget={} current:{}", target,yaw)
    r.sleep()
