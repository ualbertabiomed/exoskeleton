#!/usr/bin/python
import rospy
from joint import *
from std_msgs.msg import String

def callback(data):
    cmd = data.data
    rospy.loginfo(cmd)
    print(cmd)
    global j
    j = Joint()
    if (cmd == "c"):
        j.calibrate(True)
    if (cmd == "t"):
        j.sinusoidal_test_move(True)
    elif (cmd == "g"):
        rospy.loginfo(j.get_position())
    else:
        j.set_position(int(cmd))

def listener():
    rospy.init_node("Listener", anonymous=True)
    pub = rospy.Subscriber("term_cmd", String, callback)
    rospy.spin()

if __name__ == '__main__':
    print("Test")
    listener()
