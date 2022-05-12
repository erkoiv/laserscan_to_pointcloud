#!/usr/bin/env python3

import rospy 
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud

rospy.init_node("client_test")
rospy.wait_for_service("assemble_scans")
assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
pub = rospy.Publisher ("/pointcloud", PointCloud, queue_size=1)
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
        rospy.loginfo_throttle(1, len(resp.cloud.points))
    else:
        stale=True
        rospy.logwarn("USING STALE POINTCLOUD DATA")
    pub.publish (resp.cloud)
    r.sleep()