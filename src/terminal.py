#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String

class terminal():

    def __init__(self):
        self.cmd_string = "cmds:\np <position>\nconfigs\nlimit <vel/cur> <val>\nerrors\n"

    def talker():
            rospy.init_node("P", anonymous=True)
            self.pub = rospy.Publisher("term_channel", String, queue_size=10)
            self.rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                cmd = raw_input(self.main_menu)
                #####
                rospy.loginfo(cmd)
                try:
                    pub.publish(cmd)
                except NameError:
                    print("ERROR: Did not publish; Incorect type")
                rate.sleep()

if __name__ == '__main__':
    t = terminal()
    t.talker()
