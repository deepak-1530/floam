# read the numpy file and publish the trajectory

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

floamPath = "/home/deepak/IIITD/catkin_ws/src/floam/data/posesFloam025_TF.npy"
mapperPath = "/home/deepak/IIITD/catkin_ws/src/loam_velodyne/data/posesMapper.npy"

pathFloam = Path()
pathMapper  = Path()

rospy.init_node("traj_comparison_floam_mapper")
floamPub = rospy.Publisher("/floam_traj", Path, 100)
mapperPub = rospy.Publisher("/mapper_traj", Path, 100)

floam = np.load(floamPath)
mapper = np.load(mapperPath)

for i in range(floam.shape[0]):
    pFloam = PoseStamped()
    pMapper = PoseStamped()

    pFloam.pose.position.x = floam[i,1]
    pFloam.pose.position.y = floam[i,2]
    pFloam.pose.position.z = floam[i,3]
    pFloam.pose.orientation.w = 1.0

    pMapper.pose.position.x = mapper[i,1]
    pMapper.pose.position.y = mapper[i,2]
    pMapper.pose.position.z = mapper[i,3]
    pMapper.pose.orientation.w = 1.0

    pathFloam.poses.append(pFloam)
    pathMapper.poses.append(pMapper)

    pathFloam.header.frame_id = "/map"
    pathMapper.header.frame_id = "/map"

    pathFloam.header.stamp = rospy.Time.now()
    pathMapper.header.stamp = rospy.Time.now()

while True:
    floamPub.publish(pathFloam)
    mapperPub.publish(pathMapper)