# subscribe to the topic /camera/odom/sample
# store the points in a numpy array

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

coords = []

def odomCb(msg):
    pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    print(pose)
    coords.append(pose)
    coords_ = np.array(coords).reshape(-1,3)
    print(coords_)
    np.save("t265_data.npy", coords_)

def getData():
    rospy.init_node("t265_data_collect")
    rospy.Subscriber("/camera/odom/sample", Odometry, odomCb)
    rospy.spin()

getData()