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
        self.duration = cTime - self.initTime
        self.initTime = self.cTime
        self.err = err
        return calcP() + calcI() + calcD()

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
        return self.Kd*self.derv
