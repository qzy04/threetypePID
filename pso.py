import numpy as np
from pyswarms.single.global_best import GlobalBestPSO
import matplotlib.pyplot as plt
import PID
# from PID import PositionalPID


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
def rosenbrock_with_args(x, target_qzy):
#     x1 = x[:, 0]
#     x2 = x[:, 1]
#     x3 = a - x[:, 0]
#     f = (a - x[:, 0]) ** 2 + b * (x[:, 1] - x[:, 0] ** 2) ** 2 + c
    PIDErrADD = 0
    ErrLast = 0
    # Err = np.zeros(50)
    y = np.zeros(50)
    s = np.zeros(50)
    PositionalYaxis = np.zeros([40,50])
    Err = np.zeros([40,50])

    PositionalPid = PID.PositionalPID(x[:, 0], x[:, 1], x[:, 2])


    for i in range(50):
        PositionalPid.SetStepSignal(100.2)
        PositionalPid.SetInertiaTime(3, 0.1)
        PositionalYaxis[:,i] = PositionalPid.SystemOutput
        # PositionalYaxis.append(PositionalPid.SystemOutput)
        # PositionalPid = PID.PositionalPID(x[:, 0], x[:, 1], x[:, 2])
        # Err[i] = target_qzy - PositionalPid.PIDOutput
    Err = target_qzy - PositionalYaxis

    # Err = target_qzy - PositionalYaxis
    # for j in range(len(Err)):
    #     # s[j] = (y[j] - target_qzy) ** 2
    #     s[j] = Err[] ** 2
    f = np.sum(Err ** 2)

    #
    # ErrLast = Err  # 本次误差值赋值成上一次
    return f