#!/usr/bin/env python
"""
Node for generating data for testing that mimics what would come from the Sensors
"""

import rospy

class ArduinoSim:

    def __init__(self):
        self.pub = rospy.Publisher()
    
    def talker(self):
        rospy.init_node("arduino_sim")

    def generate_fsr(self):
        pass

    def generate_flex(self):
        pass

    def generate_imu(self):
        pass