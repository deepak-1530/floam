# subscribe to odometry topic /integrated_to_init

import math
import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf
import os
import ros_numpy


trajectory =  Path()
trajPub = rospy.Publisher("/slamTrajectoryFloam", Path, queue_size=1)
markerPub = rospy.Publisher("/slamTrajMarkerFloam", MarkerArray, queue_size=1)

mapPath     = '/home/deepak/IIITD/catkin_ws/src/floam/maps'
mapFileName = 'floamMap_Test_12Nov_bag.npy'

prevWayPoint = []
currWayPoint = []

count = 0
poses = []
mavrosPoses = []
timeStampPrev = 0

times = []
Yaws  = []

T_Lidar2Local = np.zeros((3,4))

def mapCallback(msg):
    # convert to numpy array and save

    pcNp = ros_numpy.numpify(msg)

    pointsNp = np.zeros((pcNp.shape[0], 3))

    pointsNp[:,0] = pcNp['x']
    pointsNp[:,1] = pcNp['y']
    pointsNp[:,2] = pcNp['z']
    
    np.save(os.path.join(mapPath, mapFileName), pointsNp)
    print("Map Saved as numpy array")
    

def eulerFromQuaternion(x, y, z, w):
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

def computeCb(msg):
    global times 

    times.append(msg.data)

    Times = np.array(times)
    np.save('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30_times_30cm_20kFeats.npy', Times)

def mavrosPoseCb(msg):
    print(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    global mavrosPoses
    print("mavros poses taken")
    data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    mavrosPoses.append(data)#, msg.pose.orientation.x. msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    np.save("mavrosPoses_test_floam_12nov_football_track_bag.npy", np.array(mavrosPoses))

def mavrosCb(msg):
    global Yaws

    # convert the quaternion to the euler angles
    quats = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quats)
    yaw   = euler[0]
    Yaws.append(euler) # this is yaw in radians
    np.save('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30_yaws_30cm.npy', np.array(Yaws))

def odomCb(msg):
    global prevWayPoint, currWayPoint
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
    marker.color.g = 1.0
    marker.color.b = 0
    marker.pose.orientation.w = 1.0
    marker.pose.position = p.pose.position
    marker.id = count
    marker.ns = str(count)
    m.markers.append(marker)

    trajectory.poses.append(p)
    trajPub.publish(trajectory)
    markerPub.publish(m)


    quaternion = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    poses.append([timeStamp, p.pose.position.x, p.pose.position.y, p.pose.position.z, euler[0], euler[1], euler[2]])
    print [timeStamp, p.pose.position.x, p.pose.position.y, p.pose.position.z, euler[0], euler[1], euler[2]]
    posesNp = np.asanyarray(poses)
    #np.save("/home/deepak/IIITD/catkin_ws/src/data/posesFloam025_transformedPC_TF_new_2.npy", posesNp)
    #np.save("/home/deepak/IIITD/catkin_ws/src/data/posesFloam_r100_40.npy", posesNp) ORIGINAL

    #np.save('posesFloam_Bag_dthresh_30_6thOctober.npy', posesNp)
    #np.save('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30_30cm_thresh_20kFeats.npy', posesNp) # with 35 metre threshold
    
    #np.save('test_12Nov_bag_football_track_test_reloc_10Nov.npy', posesNp)
    
    if count!=0:
        print timeStamp - timeStampPrev

    timeStampPrev = timeStamp
    count += 1
    
if __name__=="__main__":
    rospy.init_node("trajectory_node_floam_tf")
    print("Here")
    odomSub = rospy.Subscriber("/odom", Odometry, odomCb)
    computeTimeSub = rospy.Subscriber('/computeTime', Float64, computeCb)
    mavrosCb       = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mavrosPoseCb)
    #yawMavros      = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mavrosCb)
    mapCb          = rospy.Subscriber('/map', PointCloud2, mapCallback)
    rospy.spin()