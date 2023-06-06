#!/usr/bin/env python3

# a ros node that sends commands to the robot
# send to /tx with dim[0].label = "suav_2" and data = [1.0, 2.0]

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def talker():
    pub = rospy.Publisher('/tx', Float32MultiArray, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ip = input("Enter command: ")
        dim = MultiArrayDimension()
        dim.label = "suav_2"
        dim.size = 2
        dim.stride = 1
        msg = Float32MultiArray()
        msg.layout.dim.append(dim)
        msg.data = [1.0, 2.0]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass