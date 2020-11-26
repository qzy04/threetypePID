import PID
import matplotlib.pyplot as plt  # 导入绘图库

plt.figure(1)  # 创建图表1
plt.figure(2)  # 创建图表2
plt.figure(3)


# 测试PID程序
def TestPID(P, I, D):
    IncrementalPid = PID.IncrementalPID(P, I, D)
    PositionalPid = PID.PositionalPID(P, I, D)
    c_PID = PID.PID(P, I, D)
    IncrementalXaxis = [0]
    IncrementalYaxis = [0]
    PositionalXaxis = [0]
    PositionalYaxis = [0]
    c_PIDXaxis = [0]
    c_PIDYaxis = [0]

    for i in range(1, 500):
        # 增量式
        IncrementalPid.SetStepSignal(100.2)  # 设置设定值
        IncrementalPid.SetInertiaTime(3, 0.1)  # 设置惰性时间和采样时间
        IncrementalYaxis.append(IncrementalPid.SystemOutput)  # 制作Y轴列表
        IncrementalXaxis.append(i)  # x轴列表

        # 位置式
        PositionalPid.SetStepSignal(100.2)
        PositionalPid.SetInertiaTime(3, 0.1)
        PositionalYaxis.append(PositionalPid.SystemOutput)
        PositionalXaxis.append(i)

        # 连续PID
        c_PID.update(100.2)
        c_PID.SetInertiaTime(3, 0.1)
        c_PIDYaxis.append(c_PID.output)
        c_PIDXaxis.append(i)

    plt.figure(1)  # 选择图表1
    plt.plot(IncrementalXaxis, IncrementalYaxis, 'r')  # 图标数值
    plt.xlim(0, 120)  # X轴数值
    plt.ylim(0, 140)  # Y轴数值
    plt.title("IncrementalPID")  # 设定标题

    plt.figure(2)  # 选择图表2
    plt.plot(PositionalXaxis, PositionalYaxis, 'b')
    plt.xlim(0, 120)
    plt.ylim(0, 140)
    plt.title("PositionalPID")

    plt.figure(3)  # 选择图表1
    plt.plot(c_PIDXaxis, c_PIDYaxis, 'g')  # 图标数值
    plt.xlim(0, 120)  # X轴数值
    plt.ylim(0, 600)  # Y轴数值
    plt.title("PID")  # 设定标题

    plt.show()  # 图标显示


if __name__ == "__main__":
    TestPID(4.5, 0.5, 0.1)  # 设定PID值