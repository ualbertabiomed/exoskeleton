from collections import namedtuple

gains = namedtuple('gains', 'kp ki kd')  # namedtuple is immutable


class Joint:
    '''
        @param: ODrive oobject, list gains, boolean calibrate
    '''

    def __init__(self):
        '''
            class attributes:
            ODrive = oobject;
            g.kp = gains[0];
            g.ki = gains[1];
            g.kd = gains[2];
            target_position;
            current_position;

            check whether calibration needs to happen during initialization
        '''
        pass

    '''
        @brief: runs initial calibration of the brushless DC motor and position encoder 
    '''

    def calibrate(self):
        pass

    '''
        @brief: use set_position and get_position for pd control
    '''

    def update(self):
        pass

    '''
        @param: float pos
    '''

    def set_position(self):
        pass

    '''
        @return: float pos
    '''

    def get_position(self):
        pass
