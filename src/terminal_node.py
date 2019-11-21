#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String

class terminal():

    main_menu = "Press:\nc: Calibrate\nconfigs\nm: Control commands\ne: Dump errors\nr: Reset\nq: Quit\n"
    config_menu = "Select Config Setting\n1: Current Limit\n2: Velocity Limit\nm: dump motor config\ne: dump encoder config"
    control_menu = "Press:\n1: Execute test move\n2: Simple Position Control \n3: Position Trajectory Control\n4: Position Circular Control\n5: Simple Velocity Control\n6: Ramped Velocity Control\n7: Current Control\n8: Voltage Control\np: Live Plotter\ns: stop movement\nb: back\n"

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
