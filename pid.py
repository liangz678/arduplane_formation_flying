import time


class PID(object):

    def __init__(self, p=0, i=0, d=0, **kwargs):

        self._get_time = kwargs.pop('get_time', None) or time.time

        # initialze gains
        self.Kp = p
        self.Ki = i
        self.Kd = d

        # The value the controller is trying to get the system to achieve.
        self._target = 0

        # initialize delta t variables
        self._prev_tm = self._get_time()

        self._prev_feedback = 0

        self._error = None

    @property
    def error(self):
        return self._error

    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, v):
        self._target = float(v)

    def __call__(self, feedback, curr_tm=None):

        # Calculate error.
        error = self._error = feedback - self._target

        # Calculate time differential.
        if curr_tm is None:
            curr_tm = self._get_time()
        dt = curr_tm - self._prev_tm

        # Initialize output variable.
        alpha = 0

        # Add proportional component.
        alpha -= self.Kp * error

        # Add integral component.
        alpha -= self.Ki * (error * dt)

        # Add differential component (avoiding divide-by-zero).
        if dt > 0:
            alpha -= self.Kd * ((feedback - self._prev_feedback) / float(dt))



        #print(alpha, -1*self.Kp*error , -1*self.Ki * error * dt,-self.Kd*((feedback - self._prev_feedback) / float(dt)) )
        # Maintain memory for next loop.
        self._prev_tm = curr_tm
        self._prev_feedback = feedback



        return alpha
