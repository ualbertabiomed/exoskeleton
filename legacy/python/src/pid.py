import rospy
import matplotlib.pyplot as plt


"""
NOTE:
Very difficult to test.  The time will vary slightly (i.e. by 0.00001 normally)
This makes the end calculation vary a little as well (i.e. by 0.00001).
Compounded over time, these differences will become larger.


"""


class PID:
    def __init__(self, Kp=0.001, Ki=0, Kd=0):
        ''' init the PID control, initialize all variables we will need
        Parameters:
            Kp (float):     The value for the constant Kp, default to 0.001
            Ki (float):     The value for the constant Ki, default to 0
            Kd (float):     The value for the constant Kd, default to 0

        Returns:
            None
        '''
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



if __name__ == '__main__':
    testPID()
