#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String

class terminal():
    """
    Node to send the system commands from terminal on controller
    """

    def __init__(self):
        self.cmd_string = "cmds:\np <position>\nconfigs\nlimit <vel/cur> <val>\nerrors\ncalibration\n"


    def talker(self):
            rospy.init_node("terminal", anonymous=True)
            self.pub = rospy.Publisher("term_channel", String, queue_size=10)
            self.rate = rospy.Rate(1)
            rospy.loginfo("Type this following strings to execute certain command:\nposition <val>: p<val>\nconfigs<motor/encoder>: f<m/e>\nlimit <vel/cur> <val> = l<v/c><val>\ncalibration<full/user-driven-index> = c <f/u> \nerrors : e\n")
            while not rospy.is_shutdown():
                #cmd = raw_input(self.main_menu)
                cmd = raw_input("Enter terminal command: ")
                rospy.loginfo(cmd)
                try:
                    self.pub.publish(cmd)
                except NameError:
                    print("ERROR: Did not publish; Incorrect type")
                self.rate.sleep()

if __name__ == '__main__':
    t = terminal()
    t.talker()
