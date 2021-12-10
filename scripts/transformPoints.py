import numpy as np
import ros_numpy
import math
from sensor_msgs.msg import PointCloud2
import rospy


# subscribe to /velodyne_points -> rotate acw by 0.07 rads -> republish

def pointsCallback(msg):
    print("here")
    pc = ros_numpy.numpify(msg)

    if pc.size > 0:
        height = pc.shape[0]
        width = pc.shape[1]
        np_points = np.zeros((height * width, 3), dtype=np.float32)
        np_points[:, 0] = np.resize(pc['x'], height * width)
        np_points[:, 1] = np.resize(pc['y'], height * width)
        np_points[:, 2] = np.resize(pc['z'], height * width)
    
    theta = 20.0*3.14/180.0

    R = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])

    transformedPts        = np.zeros(np_points.shape)
    transformedPts[:,0:2] = np.dot(R, np_points[:,0:2].T).T
    transformedPts[:,2]   = np_points[:,2]
    
    print(transformedPts.shape)

    tfPoints = PointCloud2()

    
    

if __name__=="__main__":
    rospy.init_node("rotate_pointcloud", anonymous=True)
    rospy.Subscriber('/synced/velodyne_points', PointCloud2, pointsCallback)
    rospy.spin()

