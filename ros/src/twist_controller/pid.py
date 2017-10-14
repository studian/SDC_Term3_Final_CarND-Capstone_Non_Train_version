
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0
        self.last_error = 0.

    def freeze(self):
        self.last_error = 0.

    def step(self, error, sample_time, mn=MIN_NUM, mx=MAX_NUM):
        self.last_int_val = self.int_val
        self.min = mn
        self.max = mx

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
