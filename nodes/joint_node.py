#!/usr/bin/python
import rospy
from joint import *
from std_msgs.msg import String

def callback(data):
    cmd = data.data
    rospy.loginfo(cmd)
    print(cmd)
    if (cmd == "c"):
        j = Joint()
    # if cmd == 10:
    #     rospy.loginfo(cmd)
    #     #j.calibrate(True)

def listener():
    rospy.init_node("Listener", anonymous=True)
    pub = rospy.Subscriber("term_cmd", String, callback)
    rospy.spin()

if __name__ == '__main__':
    print("Test")
    listener()
