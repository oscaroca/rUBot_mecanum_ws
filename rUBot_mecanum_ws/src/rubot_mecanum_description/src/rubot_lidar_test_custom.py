#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print("-------------------------------------------------------")
    print("Laser_factor: "+ str(len(msg.ranges)/360) + " beams/deg")
    bd = len(msg.ranges)/360
    print ("Number of scan points: "+ str(len(msg.ranges)))
    # values at 0 degrees
    print ("Distance at 0deg: " + str(msg.ranges[round(0*bd)]))
    # values at 90 degrees
    print ("Distance at 90deg: " + str(msg.ranges[round(90*bd)]))
    # values at 180 degrees
    print ("Distance at 180deg: " + str(msg.ranges[round(180*bd)]))
    # values at 270 degrees
    print ("Distance at 270deg: " + str(msg.ranges[round(270*bd)]))
    # values at 360 degrees
    print ("Distance at 360deg: " + str(msg.ranges[round(360*bd)-1]))

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
