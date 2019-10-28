#!/usr/bin/env python
import rospy
import smach
import smach_ros
import signal
import math
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float32

global shutdown_requested

"""
NOTE:
Very difficult to test.  The time will vary slightly (i.e. by 0.00001 normally)
This makes the end calculation vary a little as well (i.e. by 0.00001).
Compounded over time, these differences will become larger.
"""

class Wait4both(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','wait4imu', 'wait4odrive'])
        return

    def execute(self, userdata):
        global shutdown_requested
        self.wait_pub = rospy.Publisher('wait_channel', String, queue_size=10)
        self.imu_sub = rospy.Subscriber('imu_channel', Float32, self.imu_callback)
        self.odrive_sub = rospy.Subscriber('odrive_channel', Float32, self.odrive_callback)
        self.imuVal = None
        self.odriveVal = None
        rate = rospy.Rate(10) # 10hz
        msg = "in wait4both"

        while not shutdown_requested:
            if self.imuVal != None:
                unregisterSubs(self)
                return 'wait4odrive'
            if self.odriveVal != None:
                unregisterSubs(self)
                return 'wait4imu'
            self.wait_pub.publish(msg)
            rate.sleep()

        self.unregisterSubs()
        return 'done'

    def unregisterSubs(self):
        self.wait_pub.unregister()
        self.imu_sub.unregister()
        self.odrive_sub.unregister()
        return

    def imu_callback(self, data):
        self.imuVal = data.data
        return

    def odrive_callback(self, data):
        self.odriveVal = data.data
        return

class Wait4imu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait4both', 'pidcalc'])
        return

    def execute(self, userdata):
        global shutdown_requested
        self.wait_pub = rospy.Publisher('wait_channel', String, queue_size=10)
        self.imu_sub = rospy.Subscriber('imu_channel', Float32, self.imu_callback)
        self.imuVal = None
        rate = rospy.Rate(10) # 10hz
        msg = "in wait4imu"
        startTime = rospy.time()

        while not shutdown_requested:
            if self.imuVal != None:
                self.unregisterSubs()
                return 'pidcalc'
            if rospy.time() > startTime + 5:
                self.unregisterSubs()
                return 'wati4both'
            self.wait_pub.publish(msg)
            rate.sleep()

        self.unregisterSubs()
        return 'done'

    def unregisterSubs(self):
        self.wait_pub.unregister()
        self.imu_sub.unregister()
        return

    def imu_callback(self, data):
        self.imuVal = data.data
        return

class Wait4odrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait4both', 'pidcalc'])
        return

    def execute(self, userdata):
        global shutdown_requested
        self.wait_pub = rospy.Publisher('wait_channel', String, queue_size=10)
        self.imu_sub = rospy.Subscriber('imu_channel', Float32, self.imu_callback)
        self.imuVal = None
        rate = rospy.Rate(10) # 10hz
        msg = "in wait4odrive"
        startTime = rospy.time()

        while not shutdown_requested:
            if self.imuVal != None:
                self.unregisterSubs()
                return 'pidcalc'
            if rospy.time() > startTime + 5:
                self.unregisterSubs()
                return 'wati4both'
            self.wait_pub.publish(msg)
            rate.sleep()

        self.unregisterSubs()
        return 'done'

    def unregisterSubs(self):
        self.wait_pub.unregister()
        self.imu_sub.unregister()
        return

    def imu_callback(self, data):
        self.imuVal = data.data
        return

class Pidcalc(smach.State):
    def __init__(self, Kp=0.001, Ki=0, Kd=0):
        ''' init the PID control, initialize all variables we will need
        Parameters:
            Kp (float):     The value for the constant Kp, default to 0.001
            Ki (float):     The value for the constant Ki, default to 0
            Kd (float):     The value for the constant Kd, default to 0

        Returns:
            None
        '''
        smach.State.__init__(self, outcomes=['wait4both'])
        self.err = None
        self.initTime = rospy.get_time()
        self.cTime = None
        self.duration = None
        self.Kp = None
        self.Ki = None
        self.Kd = None
        self.err = None
        self.prevErr = None
        self.totalErr = 0
        self.derv = None
        return

    def execute(self, userdata):
        global shutdown_requested
        self.stateSwitch = None
        err = 0 # TODO: EDIT THIS
        self.calcPID(self, err)
        return 'wait4both'

    def setConstants(self, Kp, Ki, Kd):
        ''' Set the values of each constant
        Parameters:
            Kp (float):     The value to set self.Kp to
            Ki (float):     The value to set self.Ki to
            Kd (float):     The value to set self.Kd to

        Returns:
            None
        '''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        return

    def calcPID(self, err):
        ''' Calculate the adjustment needed via PID
        Parameters:
            err (float):    The current err from where our current position is
                            compared to our goal of where we want to be
        Returns:
            float:          the adjustments needed to move to the current goal

        Formula:
             Kp * Current Error + Ki * Integral + Kd * Derivative
        '''
        self.cTime = rospy.get_time()
        self.duration = self.cTime - self.initTime
        self.initTime = self.cTime
        self.err = err
        return self.calcP() + self.calcI() + self.calcD()

    def calcP(self):
        ''' Calculate the P portion of PID
        Parameters:
            None
        Returns:
            float:          The P portion of PID
        Formula:
            Kp * Current Error
        '''
        return self.Kp * self.err

    def calcI(self):
        ''' Calculate the I portion of PID
        Parameters:
            None
        Returns:
            float:          The I portion of PID
        Formula:
            integral += Current Error * Duration
            Ki * integral
        '''
        self.totalErr += self.err*self.duration
        return self.Ki*self.totalErr

    def calcD(self):
        ''' Calculate the D portion of PID
        Parameters:
            None
        Returns:
            float:          The D portion of PID
        Formula:
            Kd * (Current Error - Previous Error) / Duration
        '''
        if not self.prevErr == None:
            self.derv = float((self.err-self.prevErr)/self.duration)
        else:
            self.derv = 0
        self.prevErr = self.err
        return self.Kd*self.derv

def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def testPID():
    '''NOTE:
    This can be played with to see the difference in the constants
    Ki should be approx. 1e5 and Kd should be approx. 1e-5
    This is because duration is approx. 1e-5, and Ki is multplied by duration
    and Kd is divded by duration.  The orders balance each other
    '''
    rospy.init_node('PID_Tester')
    controller = PID()
    controller.setConstants(0.55, 20000, 0.000005)
    goal = 0.3
    curPos = 0.0
    counter = 0
    posQueue = list()
    posQueue.append(curPos)
    while counter < 100:
        curPos += controller.calcPID(goal-curPos)
        posQueue.append(curPos)
        # print("curPos: {} \t err: {} \t duration: {}".format(curPos, goal-curPos, controller.duration))
        # print("P: {} \t I: {} \t D: {}".format(controller.Kp*controller.err,
                # controller.Ki*controller.totalErr, controller.Kd*controller.derv))
        counter += 1
        if curPos > 100:
            break
    plt.clf()
    plt.plot([x for x in range(len(posQueue))], posQueue)
    plt.plot([x for x in range(len(posQueue))], [goal for x in range(len(posQueue))])
    plt.show()
    return

def main():
    global button_start
    global shutdown_requested
    button_start = False
    shutdown_requested = False

    rospy.init_node('pid_node')

    # Create done outcome which will stop the state machine
    sm_pid = smach.StateMachine(outcomes=['DONE'])

    with sm_pid:
        smach.StateMachine.add('WAIT4BOTH', Wait4both(),
                               transitions={'wait4imu': 'WAIT4IMU',
                                            'wait4odrive': 'WAIT4ODRIVE',
                                            'done': 'DONE'})
        smach.StateMachine.add('WAIT4IMU', Wait4imu(),
                               transitions={'wait4both': 'WAIT4BOTH',
                                            'pidcalc': 'PIDCALC'})
        smach.StateMachine.add('WAIT4ODRIVE', Wait4odrive(),
                               transitions={'wait4both': 'WAIT4BOTH',
                                            'pidcalc': 'PIDCALC'})
        smach.StateMachine.add('PIDCALC', Pidcalc(),
                               transitions={'wait4both': 'WAIT4BOTH'})

    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('PID_server', sm_pid, 'STATEMACHINE')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_pid.execute()

    # Stop server
    sis.stop()

if __name__ == '__main__':
    main()
