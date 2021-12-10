# Subscribe to poses obtained from GPS in MAVROS
import numpy as np
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import PoseStamped


# Subscribe to the topic mavros/local_position/pose and store the x,y,z coordinates as posesGps.npy
# Plot the graph of heights at various time instances

poses = []



def poseCb(msg):
    poses.append([msg.pose.position.x,msg.pose.position.y, msg.pose.position.z])
    posesNp = np.array(poses).reshape(-1,3)
    np.save("/home/deepak/IIITD/catkin_ws/src/data/posesGps.npy", posesNp)

def getData():
    rospy.init_node("gps_data")
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCb)
    rospy.spin()

print("Started ...")
getData()