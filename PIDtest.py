from pyswarms.utils.plotters import (plot_cost_history, plot_contour, plot_surface)

import PID
import numpy as np
import matplotlib.pyplot as plt  # 导入绘图库
from pyswarms.single.global_best import GlobalBestPSO
from IPython.display import Image
from pyswarms.utils.functions import single_obj as fx
from pyswarms.utils.plotters.formatters import Designer


from pso import rosenbrock_with_args
from pyswarms.utils.plotters.formatters import Mesher

# plt.figure(1)  # 创建图表1
# plt.figure(2)  # 创建图表2
# plt.figure(3)
# plt.figure(4)
# plt.figure(5)
# plt.figure(6)


# 测试PID程序
def TestPID(P, I, D):
    # global target
    # target = 100
    IncrementalPid = PID.IncrementalPID(P, I, D)
    PositionalPid = PID.PositionalPID(P, I, D)
    c_PID = PID.PID(P, I, D)
    IncrementalXaxis = [0]
    IncrementalYaxis = [0]
    Incremental_U = [0]
    PositionalXaxis = [0]
    PositionalYaxis = [0]
    Positional_U = [0]
    c_PIDXaxis = [0]
    c_PIDYaxis = [0]
    c_PIDYaxis_U = [0]

    for i in range(1, 500):
        # 增量式
        # IncrementalPid.SetStepSignal(target)  # 设置设定值
        IncrementalPid.SetStepSignal(100.2)
        IncrementalPid.SetInertiaTime(3, 0.1)  # 设置惰性时间和采样时间
        IncrementalYaxis.append(IncrementalPid.SystemOutput)  # 制作Y轴列表
        IncrementalXaxis.append(i)  # x轴列表
        Incremental_U.append(IncrementalPid.PIDOutput)

        # 位置式
        # PositionalPid.SetStepSignal(target)
        PositionalPid.SetStepSignal(100.2)
        PositionalPid.SetInertiaTime(3, 0.1)
        PositionalYaxis.append(PositionalPid.SystemOutput)
        PositionalXaxis.append(i)
        Positional_U.append(PositionalPid.PIDOutput)

        # 连续PID
        # c_PID.update(target)
        c_PID.update(100.2)
        c_PID.SetInertiaTime(3, 0.1)
        c_PIDYaxis.append(c_PID.SystemOutput)
        c_PIDXaxis.append(i)
        c_PIDYaxis_U.append(c_PID.output)

    plt.figure(1)
    plt.subplot(221)  # 选择图表1
    plt.plot(IncrementalXaxis, IncrementalYaxis, 'r')  # 图标数值
    plt.xlim(0, 120)  # X轴数值
    # plt.ylim(0, 140)  # Y轴数值
    plt.title("IncrementalPID")  # 设定标题

    plt.subplot(222)  # 选择图表2
    plt.plot(PositionalXaxis, PositionalYaxis, 'b')
    plt.xlim(0, 120)
    # plt.ylim(0, 140)
    plt.title("PositionalPID")

    plt.subplot(223)  # 选择图表1
    plt.plot(c_PIDXaxis, c_PIDYaxis, 'g')  # 图标数值
    plt.xlim(0, 120)  # X轴数值
    # plt.ylim(0, 600)  # Y轴数值
    plt.title("PID")  # 设定标题
    plt.tight_layout()

    plt.figure(2)  # 选择图表2
    plt.subplot(221)
    plt.plot(PositionalXaxis, Incremental_U, 'r')
    plt.xlim(0, 120)
    # plt.ylim(0, 140)
    plt.title("IncrementalPID Control Input")
    #
    # plt.figure(5)  # 选择图表2
    plt.subplot(222)
    plt.plot(PositionalXaxis, Positional_U, 'b')
    plt.xlim(0, 120)
    # plt.ylim(0, 140)
    plt.title("PositionalPID Control Input")
    #
    # plt.figure(6)  # 选择图表2
    plt.subplot(223)
    plt.plot(PositionalXaxis, c_PIDYaxis_U, 'g')
    plt.xlim(0, 120)
    # plt.ylim(0, 140)
    plt.title("PID Control Input")
    plt.tight_layout()

    # plt.show()  # 图标显示


if __name__ == "__main__":
    # global target
    # TestPID(4.5, 0.5, 0.1)  # 设定PID值
    # TestPID(5, 1, 0)

    options = {'c1': 0.5, 'c2': 0.3, 'w': 0.9}
    # x_max = 5 * np.ones(3)
    x_max = np.array([5, 3, 1])
    x_min = 0 * x_max
    bounds = (x_min, x_max)
    particle = 30
    target = 20
    optimizer = GlobalBestPSO(n_particles=particle, dimensions=3, options=options, bounds=bounds)
    cost, pos = optimizer.optimize(rosenbrock_with_args, 1000, n=particle, t=100, target_qzy=target)
    # cost, pos = optimizer.optimize(rosenbrock_with_args, 1000, a=1, b=100, c=0)

    TestPID(pos[0], pos[1], pos[2])  # 设定PID值

    plot_cost_history(cost_history=optimizer.cost_history)

    # f = rosenbrock_with_args(x, particle, 100, target)
    # m = Mesher(func=fx.sphere)
    # animation = plot_contour(pos_history=optimizer.pos_history, mesher=m)
    # animation.save('plot0.gif', writer='imagemagick', fps=10)
    # Image(url='plot0.gif')

    # pos_history_3d = m.compute_history_3d(optimizer.pos_history)
    # d = Designer(limits=[(-1, 1), (-1, 1), (-0.1, 1)], label=['x-axis', 'y-axis', 'z-axis'])
    # animation3d = plot_surface(pos_history=pos_history_3d,  # Use the cost_history we computed
    #                            mesher=m, designer=d,  # Customizations
    #                            mark=(0, 0, 0))  # Mark minima
    # # animation3d.save('plot1.gif', writer='imagemagick', fps=10)
    # Image(url='plot1.gif')
    plt.show()
