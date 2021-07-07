# subscribe to odometry topic /integrated_to_init

import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

trajectory =  Path()
trajPub = rospy.Publisher("/slamTrajectoryFloam", Path, queue_size=1)
markerPub = rospy.Publisher("/slamTrajMarkerFloam", MarkerArray, queue_size=1)


count = 0
poses = []
timeStampPrev = 0

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)*(180.0/3.14)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)*(180.0/3.14)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)*(180.0/3.14)
     
        return roll_x, pitch_y, yaw_z # in degrees

def odomCb(msg):
    global count
    global timeStampPrev
    p = PoseStamped()
    m = MarkerArray()
    marker = Marker()

    timeStamp = float(str(msg.header.stamp.secs)  + "." + str(msg.header.stamp.nsecs))

    trajectory.header.frame_id = msg.header.frame_id
    trajectory.header.stamp = rospy.Time.now()
    p.pose.position = msg.pose.pose.position
    p.pose.orientation = msg.pose.pose.orientation
    
    marker.header.frame_id = msg.header.frame_id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.7
    marker.scale.y = 0.7
    marker.scale.z = 0.7
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0
    marker.pose.orientation.w = 1.0
    marker.pose.position = p.pose.position
    marker.id = count
    marker.ns = str(count)
    m.markers.append(marker)

    trajectory.poses.append(p)
    trajPub.publish(trajectory)
    markerPub.publish(m)

    roll, pitch, yaw = euler_from_quaternion(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
    poses.append([timeStamp, p.pose.position.x, p.pose.position.y, p.pose.position.z, roll, pitch, yaw])
    posesNp = np.asanyarray(poses)
    np.save("/home/deepak/IIITD/catkin_ws/src/floam/data/posesFloam_vox30.npy", posesNp)
    if count!=0:
        print(f"Computation time per scan is: {timeStamp - timeStampPrev}")

    timeStampPrev = timeStamp
    count += 1
    
if __name__=="__main__":
    rospy.init_node("trajectory_node_floam")
    print("Here")
    odomSub = rospy.Subscriber("/odom", Odometry, odomCb)
    rospy.spin()

