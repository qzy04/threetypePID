import time


class PID:
    # 初始化P I D值，调用类可设置PID，也可通过下面函数单独设置某一个
    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00  # 采样时间
        self.current_time = time.time()  # 调用类时当前时间，相当于0
        self.last_time = self.current_time  # 上一次时间同上
        self.SystemOutput = 0.0
        self.LastSystemOutput = 0.0

        self.clear()  # 每次调用先清零上一次调用的计算值

    def clear(self):
        """Clears PID computations and coefficients"""
        # self.SetPoint = 0.0  # 设定值

        self.PTerm = 0.0  # 比例值无Kp
        self.ITerm = 0.0  # 积分值无Ki
        self.DTerm = 0.0  # 微分值无Kd
        self.last_error = 0.0  # 上一次误差

        # Windup Guard
        self.int_error = 0.0  # 初始误差为0，意义不大
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, SetPoint):

        error = SetPoint - self.SystemOutput  # 设定值减去反馈值，误差

        self.current_time = time.time()  # 调用类函数时当前时间
        delta_time = self.current_time - self.last_time  # 微分中的两次时间差，如果只有一次，则lasttime=0
        delta_error = error - self.last_error  # 微分中误差值

        if delta_time >= self.sample_time:  # 两次反馈值得时间间隔和采样时间如果相等
            self.PTerm = self.Kp * error  # 计算比例值部分
            self.ITerm += error * delta_time  # 计算积分部分
            # 避免第一次温度升到100摄氏度过程中积分后产生过冲，将值设为20或-20，n次后，积分的各值有正负基本不会超过20.
            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0  # 应该不等于0也没问题，微分就是和后两次的误差和时间有关
            if delta_time > 0:
                self.DTerm = delta_error / delta_time  # 计算微分部分

            # 将本次的误差和时间设置成上一次的，为下次计算做准备
            self.last_time = self.current_time
            self.last_error = error

            # PID的输出值（加上Kp Ki Kd）
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    def SetInertiaTime(self, InertiaTime, SampleTime):
        self.SystemOutput = (InertiaTime * self.LastSystemOutput + SampleTime * self.output) / (
                SampleTime + InertiaTime)
        self.LastSystemOutput = self.SystemOutput

    def setKp(self, proportional_gain):  # 比例增益
        self.Kp = proportional_gain

    def setKi(self, integral_gain):  # 积分增益
        self.Ki = integral_gain

    def setKd(self, derivative_gain):  # 微分增益
        self.Kd = derivative_gain

    def setWindup(self, windup):  # 防止积分第一次出现过冲，设置积分最大值
        self.windup_guard = windup

    def setSampleTime(self, sample_time):  # 采样时间
        self.sample_time = sample_time
#  PID控制一阶惯性系统测试程序

# *****************************************************************#
#                      位置式PID系统                              #
# *****************************************************************#
class PositionalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.PIDOutput = 0.0
        self.PIDErrADD = 0.0
        self.ErrLast = 0.0

        self.SystemOutput = 0.0
        self.LastSystemOutput = 0.0

    def SetStepSignal(self, StepSignal):
        Err = StepSignal - self.SystemOutput  # 设定值-反馈值=误差
        KpWork = self.Kp * Err  # 比例值
        self.PIDErrADD += Err
        KiWork = self.Ki * self.PIDErrADD  # 积分值
        KdWork = self.Kd * (Err - self.ErrLast)  # 微分值
        self.PIDOutput = KpWork + KiWork + KdWork  # PID输出值

        self.ErrLast = Err  # 本次误差值赋值成上一次

    def SetInertiaTime(self, InertiaTime, SampleTime):
        self.SystemOutput = (InertiaTime * self.LastSystemOutput + SampleTime * self.PIDOutput) / (
                SampleTime + InertiaTime)
        self.LastSystemOutput = self.SystemOutput


# *****************************************************************#
#                      增量式PID系统                              #
# *****************************************************************#
class IncrementalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.PIDOutput = 0.0  # PID控制器输出
        self.Error = 0.0  # 输出值与输入值的偏差
        self.LastError = 0.0
        self.LastLastError = 0.0

        self.SystemOutput = 0.0  # 系统输出值
        self.LastSystemOutput = 0.0  # 上次系统输出值

    # 设置PID控制器参数
    def SetStepSignal(self, StepSignal):
        self.Error = StepSignal - self.SystemOutput  # 误差值
        IncrementValue = self.Kp * (self.Error - self.LastError) + self.Ki * self.Error + self.Kd * (
                self.Error - 2 * self.LastError + self.LastLastError)  # 增量值
        self.PIDOutput += IncrementValue

        self.LastLastError = self.LastError
        self.LastError = self.Error

    # 设置一阶惯性环节系统  其中InertiaTime为惯性时间常数
    def SetInertiaTime(self, InertiaTime, SampleTime):
        self.SystemOutput = (InertiaTime * self.LastSystemOutput + SampleTime * self.PIDOutput) / (
                SampleTime + InertiaTime)
        self.LastSystemOutput = self.SystemOutput