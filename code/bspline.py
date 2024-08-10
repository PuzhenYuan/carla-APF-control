import matplotlib.pyplot as plt
import numpy as np
import random
import math

def basis(i, k, t, u):
    #i，k是基函数的参数，t是自变量，这里固定为0~1，u是节点的向量
    #k+1阶，k次b样条，每一段曲线会涉及k个小区间，k+1个节点数
    #涉及的区间并起来是[u_0,u_n+k]，共n+k+1个节点
    #返回B-样条的基函数
    if k == 0: #1阶0次B样条基函数定义，为0次直线
        if(t >= u[i] and t <= u[i + 1]):
            result = 1
        else:
            result = 0
    else: #递归定义
        #计算支撑区间长度
        length1 = u[i + k] - u[i]
        length2 = u[i + k + 1] - u[i + 1]
        #若length = 0，则系数权重置为0，定义0/0=0
        if length1 == 0:
            weight1 = 0
        else:
            weight1 = (t - u[i]) / length1
        if length2 == 0:
            weight2 = 0
        else:
            weight2 = (u[i + k + 1] - t) / length2
        result = weight1 * basis(i, k - 1, t, u) + weight2 * basis(i + 1, k - 1, t, u)
    return result

def b_spline(route, t): 
    #传入预被控制的点列route，参数t取值0~1，t从0
    #route中的点称为控制点，共n+1个，即索引从0~n
    #route是包含n+1个node的列表，每个node包含x,y,vx,vy,v,theta的数据，但只调用node.x与node.y
    #采用准均匀B-样条曲线，强制第一个点和最后一个点重复度为k
    #返回B-样条函数，t取值0~1
    k = 3 #采用的样条次数
    xlist = [p.x for p in route]
    ylist = [p.y for p in route]
    
    #xylist中共有n+1=len(xlist)个元素，最后的索引是n，即[p0~pn]
    n = len(xlist) - 1
    
    #欲生成m+1个u节点，[k个0,0,1/(n-k+1),~,1,k个1]
    u = []
    for j in range(0, k):
        u.append(0)
    for j in range(0, n - k + 2):
        u.append(j / (n - k + 1))
    for j in range(0, k):
        u.append(1)
    
    #B-样条求和，从0到n
    resultx = 0
    resulty = 0
    for i in range(0, n + 1):
        resultx = resultx + basis(i, k, t, u) * xlist[i]
        resulty = resulty + basis(i, k, t, u) * ylist[i]
    return resultx, resulty

def b_tangent(route, t):
    #t取值在0~1内，最好取中间的值，如0.5
    #返回该点切线与x轴夹角
    [x0, y0] = b_spline(route, t)
    [x1, y1] = b_spline(route, t + 0.01)
    dx = x1 - x0
    dy = y1 - y0
    if abs(dx) <= 0.001:
        if dx >= 0:
            dx = 0.001
        else:
            dx = -0.001
    theta = math.atan(dy/dx)
    return theta

def b_azimuth(route, t):
    #返回方位角
    [x0, y0] = b_spline(route, 0)
    [x1, y1] = b_spline(route, t)
    dx = x1 - x0
    dy = y1 - y0
    if abs(dx) <= 0.001:
        if dx >= 0:
            dx = 0.001
        else:
            dx = -0.001
    theta = math.atan(dy/dx)
    return theta

def bezier(route, t):
    #类似b_splineB样条函数，不过是贝塞尔函数
    xlist = np.array([p.x for p in route])
    ylist = np.array([p.y for p in route])
    n = len(route) - 1
    #c_in = np.math.factorial(n) / (np.math.factorial(i) * np.math.factorial(n-i))
    b_n = np.array([np.math.factorial(n) / (np.math.factorial(i) * np.math.factorial(n-i)) * t**i * (1-t)**(n-i) for i in range(0, n + 1)])
    resultx = (xlist * b_n).sum()
    resulty = (ylist * b_n).sum()
    return resultx, resulty

#测试代码
if __name__ == '__main__':
    
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    
    plt.figure()
    route = [Node(0, 1), Node(1, 3), Node(2, 4), Node(3, 3.5), Node(5, 2), Node(6, 2), Node(7, 2), Node(10, 2.5)]
    X0 = [p.x for p in route]
    Y0 = [p.y for p in route]
    plt.plot(X0, Y0)
    X1 = []
    Y1 = []
    for t in np.linspace(0, 1, 100):
        [x1, y1] = b_spline(route, t)
        X1.append(x1)
        Y1.append(y1)
    plt.plot(X1, Y1)
    t = 1/2
    x = np.linspace(0, 8, 100)
    
    [x1, y1] = b_spline(route, t)
    theta1 = b_tangent(route, t)
    k1 = math.tan(theta1)
    plt.plot(x, k1 * (x - x1) + y1)
    
    [x2, y2] = b_spline(route, 0)
    theta2 = b_azimuth(route, t)
    k2 = math.tan(theta2)
    plt.plot(x, k2 * (x - x2) + y2)

    plt.show()