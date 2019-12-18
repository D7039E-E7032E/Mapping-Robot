#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from controller.msg import Pathvector
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import math

dirx=diry=0
rang=[]
angel_inc=0
vx=0
vy=0
kv=0.
n=1
c=10
roll = pitch = yaw = 0.0
l=400


def get_dir(msg):
    global dirx,diry 
    poses=msg.poses   
    dirx=poses[1].pose.position.x-poses[0].pose.position.x
    diry=poses[1].pose.position.y-poses[0].pose.position.y
    length=math.sqrt(dirx*dirx+diry*diry)
    dirx=dirx/length*l
    diry=diry/length*l
    #print()


def get_rang (msg):
    global angel_inc, rang
    angel_inc=msg.angle_increment
    rang=msg.ranges

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print yaw
 
rospy.init_node('localnavigator_node')
 
sub = rospy.Subscriber ('/move_base/DWAPlannerROS/global_plan', Path, get_dir)
sub2= rospy.Subscriber ('/scan', LaserScan, get_rang)
sub3 = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('pathv', Pathvector, queue_size=1)
ro = rospy.Rate(20)
command =Pathvector()
 
while not rospy.is_shutdown():
    ang=pirx=piry=0
    for r in rang:
        grad=c*math.exp(-n*r)
        pirx=pirx+grad*math.cos(ang+yaw)
        piry=piry+grad*math.sin(ang+yaw)
        ang=ang+angel_inc

    vx=kv*vx+pirx
    vy=kv*vy+piry
    command.x=-vx+dirx
    command.y=-vy+diry
    pub.publish(command)
    ro.sleep()
