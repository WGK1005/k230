# pid.py - PID控制器实现

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0, sample_time=0.01, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = output_limits

        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self._last_error = 0
        self._last_output = 0
        self._last_time = None

    def __call__(self, feedback_value, current_time=None):
        error = self.setpoint - feedback_value

        if self._last_time is None:
            self._last_time = current_time if current_time else 0
            return self._last_output

        current_time = current_time if current_time else self._last_time + self.sample_time
        dt = current_time - self._last_time

        if dt < self.sample_time:
            return self._last_output

        # 比例项
        self._proportional = self.Kp * error

        # 积分项
        self._integral += self.Ki * error * dt
        # 积分限幅
        if self._min_output is not None and self._max_output is not None:
            self._integral = max(self._min_output, min(self._max_output, self._integral))

        # 微分项
        if dt > 0:
            self._derivative = self.Kd * (error - self._last_error) / dt
        else:
            self._derivative = 0

        # 计算输出
        output = self._proportional + self._integral + self._derivative

        # 输出限幅
        if self._min_output is not None and self._max_output is not None:
            output = max(self._min_output, min(self._max_output, output))

        # 更新状态
        self._last_error = error
        self._last_time = current_time
        self._last_output = output

        return output

    def reset(self):
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self._last_error = 0
        self._last_output = 0
        self._last_time = None

    @property
    def components(self):
        return (self._proportional, self._integral, self._derivative)
