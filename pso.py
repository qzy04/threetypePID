import numpy as np
from pyswarms.single.global_best import GlobalBestPSO
import matplotlib.pyplot as plt
import PID
# from PID import PositionalPID, PID


# def sum_error(target_qzy):
#     Err = target_qzy - 1.0  # 设定值-反馈值=误差
    # Err_s = Err ** 2
    # KpWork = Kp * Err  # 比例值
    # PIDErrADD += Err
    # KiWork = Ki * self.PIDErrADD  # 积分值
    # KdWork = Kd * (Err - ErrLast)  # 微分值
    # self.PIDOutput = KpWork + KiWork + KdWork  # PID输出值
    #
    # self.ErrLast = Err  # 本次误差值赋值成上一次
    # return Err

# def rosenbrock_with_args(x, a, b, c=0):
def rosenbrock_with_args(x, n, t, target_qzy):
#     x1 = x[:, 0]
#     x2 = x[:, 1]
#     x3 = a - x[:, 0]
#     f = (a - x[:, 0]) ** 2 + b * (x[:, 1] - x[:, 0] ** 2) ** 2 + c

    PositionalYaxis = np.zeros([n, t])
    c_PIDYaxis = np.zeros([n, t])

    PositionalPid = PID.PositionalPID(x[:, 0], x[:, 1], x[:, 2])
    # c_PID = PID.PID(x[:, 0], x[:, 1], x[:, 2])


    for i in range(t):
        PositionalPid.SetStepSignal(target_qzy)
        PositionalPid.SetInertiaTime(3, 0.1)
        PositionalYaxis[:, i] = PositionalPid.SystemOutput
    Err = target_qzy - PositionalYaxis

        # c_PID.update(target_qzy)
        # c_PID.SetInertiaTime(3, 0.1)
        # c_PIDYaxis[:, i] = c_PID.SystemOutput
    # Err = target_qzy - PositionalYaxis
    f = np.sum(Err ** 2)

    #
    # ErrLast = Err  # 本次误差值赋值成上一次
    return f