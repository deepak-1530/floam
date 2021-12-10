# Publish the waypoints given to planner

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def publishPoints(xPts, yPts):
    markerPublisher = rospy.Publisher("/sampled_waypoints", MarkerArray, queue_size=100)
    count = 0
    markerArray = MarkerArray()
    
    for i in range(len(xPts)):
        marker      = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp    = rospy.Time.now()
        marker.type            = marker.SPHERE
        marker.action          = marker.ADD
        marker.scale.x         = 0.7
        marker.scale.y         = 0.7
        marker.scale.z         = 0.7
        marker.color.a         = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = xPts[i]
        marker.pose.position.y = yPts[i]
        marker.pose.position.z = 0
        marker.id = count
        marker.ns = str(count)
        markerArray.markers.append(marker)
        count += 1
    
    while True:
        markerPublisher.publish(markerArray)
        print("Marker array published")
        if rospy.is_shutdown():
            break
    

if __name__=="__main__":
    rospy.init_node("Sampled_waypoints_publisher")

    pathToWaypointsX = '/home/deepak/IIITD/catkin_ws/src/floam/scripts/waypts/wayptsX.txt'
    pathToWaypointsy = '/home/deepak/IIITD/catkin_ws/src/floam/scripts/waypts/wayptsY.txt'    

    # loop through the files and get the coordinates
    xPts = []
    yPts = []

    xFile = open(pathToWaypointsX,"r")
    yFile = open(pathToWaypointsy, "r")

    xLines = xFile.read().splitlines()
    yLines = yFile.read().splitlines()
    
    for x in xLines:
        xPts.append(float(x))
    
    for y in yLines:
        yPts.append(float(y))

    # now publish these points
    publishPoints(xPts, yPts)