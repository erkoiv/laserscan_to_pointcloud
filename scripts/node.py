#!/usr/bin/env python3

import rospy 
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

rospy.init_node("client_test")
rospy.wait_for_service("assemble_scans")
assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
cloud_pub = rospy.Publisher("/pointcloud", PointCloud, queue_size=1)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
vel_cmd = Twist()

#INITIALIZE THE PLANNER HERE

planner=None #TODO

#######

r = rospy.Rate (30)
x=[]
y=[]
stale=False

while not rospy.is_shutdown():
    resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())

    if resp != None:
        x=[]
        y=[]
        stale=False
        for point in resp.cloud.points:
            x.append(point.x)
            y.append(point.y)
        rospy.logdebug_throttle(1, len(resp.cloud.points))
    else:
        stale=True
        rospy.logwarn("USING STALE POINTCLOUD DATA OR NO LASERSCAN MESSAGE RECEIVED")

    #CALL PLANNER HERE

    #TODO

    vel_cmd.linear.x=0
    vel_cmd.angular.z=0
    #######
    cmd_vel_pub.publish(vel_cmd)
    cloud_pub.publish(resp.cloud)
    r.sleep()