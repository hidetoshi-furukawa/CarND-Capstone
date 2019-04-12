
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM, i_min=-0.04, i_max=0.04):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.integral_min = i_min
        self.integral_max = i_max

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time
        if val > self.max:
            val = self.integral_max
        elif val < self.min:
            val = self.integral_min
        
        if sample_time > 0.0:
            derivative = (error - self.last_error) / sample_time
        else:
            derivative = 0.0

        val = self.kp * error + self.ki * integral + self.kd * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error
        # print('{} {}'.format(integral, self.int_val))  # DEBUG

        return val
