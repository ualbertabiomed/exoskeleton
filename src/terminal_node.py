#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String

def talker():
        pub = rospy.Publisher("term_cmd", String, queue_size=10)
        rospy.init_node("Publisher", anonymous=True)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            cmd = raw_input("Enter a command: ")
            rospy.loginfo(cmd)
            try:
                pub.publish(cmd)
            except NameError:
                print("ERROR: Did not publish; Incorect type")
            rate.sleep()

if __name__ == '__main__':
    talker()
