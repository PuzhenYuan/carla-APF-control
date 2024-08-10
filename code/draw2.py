from matplotlib import pyplot as plt
from matplotlib import animation as ani
from matplotlib import cm
import numpy as np

x1 = np.linspace(0, 2, num=100)
y1 = np.linspace(8, 10, num=100)
[X, Y] = np.meshgrid(x1, y1)
Z = 10 - X**2 - Y

fig = plt.figure(figsize=(8, 6))
point_list = []
point_list2 = []
def basis(i):
    if i < 140:
        plt.clf()
        ax = plt.axes([0.1, 0.1, 0.8, 0.8])
        ax.contourf(X, Y, Z, cmap=cm.jet, levels=np.linspace(-2, 4, 8))
        ax.set_xlabel('x-position')
        ax.set_ylabel('y-position')
        ax.set_title('generate path')
        h = ax.contourf(X, Y, Z)
        cb = plt.colorbar(h)
        cb.set_label('field potential')
        ax.axis([0, 2, 8, 10])
        
        #画当前点的位置
        x0 = 2 - i * 0.01
        y0 = 10 - x0**2 / 2
        ax.scatter(x0, y0, color='red', linewidths=10)
        y2 = 9.75 - 0.875 * x0**2 / 2
        ax.scatter(x0, y2, color='red', linewidths=10)
        
        #画红色轨迹
        x = np.linspace(x0, 2, num=100)
        ax.plot(x, 10 - x**2 / 2, color='red')
        
        ax.plot(x, 9.75 - 0.875 * x**2 / 2, color='red')
        
        #画梯度斥力
        vf = np.array([2*x0, 1]) / np.sqrt(1 + 4*x0**2) / 6
        ax.arrow(x0, y0, *vf, color='orange', head_width=0.05, head_length=0.07)
        
        vf2 = np.array([2*x0, 1]) / np.sqrt(1 + 4*x0**2) / 6
        ax.arrow(x0, y2, *vf2, color='orange', head_width=0.05, head_length=0.07)
        
        #假设指向（0，9.5）的引力
        ax.scatter(0, 9.5, color="purple", linewidths=15)
        vt = np.array([-x0, -y0+9.5]) / np.sqrt(x0**2 + (y0-9.5)**2) / 3
        vtt = np.array([-x0, -y0+9.5])
        ax.arrow(x0, y0, *vt, color='purple', head_width=0.05, head_length=0.07)
        ax.arrow(x0, y0, *vtt, color='purple', linestyle="dotted")
        
        vt2 = np.array([-x0, -y2+9.5]) / np.sqrt(x0**2 + (y2-9.5)**2) / 2.5
        vtt2 = np.array([-x0, -y2+9.5])
        ax.arrow(x0, y2, *vt2, color='purple', head_width=0.05, head_length=0.07)
        ax.arrow(x0, y2, *vtt2, color='purple', linestyle="dotted")
        
        #画合力
        vh = vf + vt
        ax.arrow(x0, y0, *vh, color='orchid', head_width=0.05, head_length=0.07)
        
        vh2 = vf2 + vt2
        ax.arrow(x0, y2, *vh2, color='orchid', head_width=0.05, head_length=0.07)
        
        #绘制虚线
        ax.arrow(x0 + vf[0], y0 + vf[1], *vt, color='orchid', linestyle="dotted")
        ax.arrow(x0 + vt[0], y0 + vt[1], *vf, color='orchid', linestyle="dotted")
        
        ax.arrow(x0 + vf2[0], y2 + vf2[1], *vt2, color='orchid', linestyle="dotted")
        ax.arrow(x0 + vt2[0], y2 + vt2[1], *vf2, color='orchid', linestyle="dotted")
        
        #绘制速度
        vv = np.array([-1, x0]) / np.sqrt((1 + x0**2)) / 3
        ax.arrow(x0, y0, *vv, color='red', head_width=0.05, head_length=0.07)
        
        vv2 = np.array([-1, 0.875*x0]) / np.sqrt((1 + x0**2 * 0.875**2)) / 3
        ax.arrow(x0, y2, *vv2, color='red', head_width=0.05, head_length=0.07)
        
        #绘制轨迹上的点
        if i % 20 == 0 and i > 0:
            point_list.append(np.array([x0, y0]))
            point_list2.append(np.array([x0, y2]))
        for point in point_list:
            ax.scatter(point[0], point[1], color='red', linewidths=10)
        for point in point_list2:
            ax.scatter(point[0], point[1], color='red', linewidths=10)
            
    elif i > 170:
        i = 140
        plt.clf()
        ax = plt.axes([0.1, 0.1, 0.8, 0.8])
        ax.contourf(X, Y, Z, cmap=cm.jet, levels=np.linspace(-2, 4, 8))
        ax.set_xlabel('x-position')
        ax.set_ylabel('y-position')
        ax.set_title('generate path')
        h = ax.contourf(X, Y, Z)
        cb = plt.colorbar(h)
        cb.set_label('field potential')
        ax.axis([0, 2, 8, 10])
        
        #画红色轨迹
        x0 = 2 - i * 0.01
        y0 = 10 - x0**2 / 2
        ax.scatter(x0, y0, color='red', linewidths=3)
        x = np.linspace(x0, 2, num=100)
        ax.plot(x, 10 - x**2 / 2, color='red', linewidth=5.0)
        
        ax.plot(x, 9.75 - 0.875 * x**2 / 2, color='red', linestyle='dashed')

animator = ani.FuncAnimation(fig, basis, interval=20)
plt.show()
