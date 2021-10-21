###############################################
# Compensate for extra rotation in pointcloud #
###############################################

import rospy
import ros_numpy 
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
import numpy as np
import utils
from std_msgs.msg import Header

# convert from PointCloud2 to numpy array
# perform transformation on the point cloud -> rotate by -15 degrees in the pitch
pcPub = rospy.Publisher("/transformed_points", PointCloud2, queue_size=1)

Rx = np.eye(3,3)
Rz = np.eye(3,3)
t  = np.zeros((3,1))

    # transform these by performing rotation
PI = 3.14

pitch = (15.0*PI)/180.0
#pitch = 0
Ry = np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0,             1,      0]       ,
                [-np.sin(pitch),0, np.cos(pitch)]
              ])
print(Ry)

T = np.hstack(((np.dot(Rx, np.dot(Ry, Rz))), t)) 

def pcCb(msg):
    # convert msg to numpy array
    # add current time in the header
    # rest everything remains the same

    pcNp = ros_numpy.numpify(msg)
    pointsNp = np.zeros((pcNp.shape[0], 3))
    pointsNp[:,0] = pcNp['x']
    pointsNp[:,1] = pcNp['y']
    pointsNp[:,2] = pcNp['z']

    print(pointsNp)
    
    pointsTf = np.dot(Ry, pointsNp.T).T

    # publish this as a pointcloud2 message
    ros_dtype = PointField.FLOAT32
    itemsize = np.dtype(np.float32).itemsize
    data = pointsTf.astype(np.float32)

    points = data
    msg = PointCloud2()

    msg.header.stamp = rospy.Time().now()

    msg.header.frame_id = "velodyne"

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
        # msg.is_dense = int(np.isfinite(points).all())
    msg.is_dense = False
    msg.data = np.asarray(points, np.float32).tostring()
    pcPub.publish(msg)
   

rospy.init_node("transform_pointCloud")
rospy.Subscriber("/velodyne_points", PointCloud2, pcCb)
rospy.spin()