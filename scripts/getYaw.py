# subscribe to the global heading topic 
# compare the performance

import rospy
import numpy as np
from std_msgs.msg import Float64

yaws = []

def yawCb(msg):
    global yaws
    print(f'Yaw is: {msg.data}')
    if msg.data <= -180:
        msg.data += 360
    if msg.data > 180:
        msg.data -= 360 
    yaws.append(msg.data)
    np.save("mavrosYaw.npy", np.array(yaws))


if __name__=="__main__":
    rospy.init_node("get_yaw")
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, yawCb)
    rospy.spin()