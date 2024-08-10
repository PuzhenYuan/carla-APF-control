import random
import numpy as np
import math
from scipy.stats import norm
import matplotlib.pyplot as plt
import time

#自编库
import field

class MassCone():
    def __init__(self, carm, node_list, S, R, E):
        #对一辆车建立一个质量锥对象
        #node_list传递一辆车之前的几个节点，从0秒到-(N-1)秒
        #S：分割因数
        #R：信任指数
        #E：前进指数
        self.carm = carm
        self.x = [node.x for node in node_list]
        self.y = [node.y for node in node_list]
        self.vx = [node.vx for node in node_list]
        self.vy = [node.vy for node in node_list]
        self.N = len(node_list) #node的数量，共N个
        self.S = S
        self.R = R
        self.E = E
        
    def predict_node(self, p1, p2):
        #预测下一时刻的X，Y，VX，VY，是p1，p2的定值函数，p1，p2取值0~1
        
        #预测均值VX，VY，使用线性拟合，直接套用公式
        
        #拟合 vx = alpha_x * t + beta_x
        x1 = -np.arange(0, self.N)
        y1 = np.array(self.vx)
        cov_x1y1 = np.mean(x1 * y1) - np.mean(x1) * np.mean(y1)
        var_x1 = np.mean(x1**2) - np.mean(x1)**2
        alpha_x = cov_x1y1 / var_x1
        beta_x = np.mean(y1) - alpha_x * np.mean(x1)
        
        #拟合 vy = alpha_y * t + beta_y
        x2 = -np.arange(0, self.N)
        y2 = np.array(self.vy)
        cov_x2y2 = np.mean(x2 * y2) - np.mean(x2) * np.mean(y2)
        var_x2 = np.mean(x2**2) - np.mean(x2)**2
        alpha_y = cov_x2y2 / var_x2
        beta_y = np.mean(y1) - alpha_y * np.mean(x1)
        
        #拟合下一个点的均值(VX, VY)和带概率的速度大小
        VX = alpha_x + beta_x
        VY = alpha_y + beta_y
        V1 = (VX**2 + VY**2)**(1/2) #下一时刻速度大小的均值，认为速度大小服从正态分布
        sigma = self.E
        V1_ran = norm.ppf(q=p1, loc=V1, scale=sigma)
        
        #下面求角度，建立极坐标，用参数axis来描述v0与x轴夹角(建系)，phi为er与v0夹角，theta为r与v0夹角，逆时针为正
        
        #计算axis
        vx0 = self.vx[0]
        vy0 = self.vy[0]
        if abs(vx0) <= 0.001:
            vx0 = 0.001
        axis = math.atan(vy0 / vx0)
        
        #计算theta
        rx = self.x[0] - self.x[1]
        ry = self.y[0] - self.y[1]
        if abs(rx) <= 0.001:
            rx = 0.001
        theta = -(math.atan(ry / rx) - axis)
        
        #计算phi，与p2有关
        list = [(self.R)**x for x in range(0, self.S + 1)]
        Sum = np.array(list).sum()
        K = np.array(list) / Sum
        k_ran = math.ceil(np.log(p2 * Sum) / np.log(self.R)) - 1
        phi_ran = k_ran * theta
        
        #计算下一时刻坐标与速度，带双重概率(p1, p2)，p1影响速度大小， p2影响角度大小
        VX_ran = V1_ran * math.cos(phi_ran + axis)
        VY_ran = V1_ran * math.sin(phi_ran + axis)
        k = 0.01 #调节的参数
        X_ran = self.x[0] + VX_ran * k
        Y_ran = self.y[0] + VY_ran * k
        
        return X_ran, Y_ran, VX_ran, VY_ran
    
    def get_field_force(self, mycarx, mycary, mycarvx, mycarvy, n1, n2):
        #构建风险场的时候，构建几个小风险场的加权和
        #n1对应p1分成n1+1份，n2对应p2分成n2+1份
        #需要构建二维数组nset
        nset = [[] for i in range(0, n1)]
        fx = 0
        fy = 0
        for i in range(0, n1):
            for j in range(0, n2):
                [x_ij, y_ij, vx_ij, vy_ij] = self.predict_node((i + 1) / (n1 + 1), (j + 1) / (n2 + 1))
                carfield_ij = field.CarPotentialField(self.carm / (n1 * n2), x_ij, y_ij, vx_ij, vy_ij)
                [fx_ij, fy_ij] = carfield_ij.get_force(mycarx, mycary, mycarvx, mycarvy)
                fx = fx + fx_ij
                fy = fy + fy_ij
        return fx, fy
    
    def get_random_force(self, mycarx, mycary, mycarvx, mycarvy):
        #构建随机单个风险场
        #p1给定均匀分布，p2给定均匀分布
        p1 = random.random()
        p2 = random.random()
        # print('p1: ' + str(p1))
        # print('p2: ' + str(p2))
        [x, y, vx, vy] = self.predict_node(p1, p2)
        carfield_ram = field.CarPotentialField(self.carm, x, y, vx, vy)
        [fx_ran, fy_ran] = carfield_ram.get_force(mycarx, mycary, mycarvx, mycarvy)
        return fx_ran, fy_ran

#测试代码
if __name__ == '__main__':
    
    class Node: #是存储路径时的结点，类似结构体
        def __init__(self, x, y, vx, vy):
            self.x = x
            self.y = y
            self.vx = vx
            self.vy = vy
            self.v = math.sqrt(math.pow(self.vx, 2) + math.pow(self.vy, 2))
            self.theta = math.atan(vy/(vx + 0.001))

        def update(self): #用来更新参数v，theta
            self.v = math.sqrt(math.pow(self.vx, 2) + math.pow(self.vy, 2))
            self.theta = math.atan(self.vy/(self.vx + 0.001))
    
    node_list = [Node(4, 4, 1, 1), Node(3, 3, 1, 1), Node(2, 2, 1, 1), Node(1, 1, 1, 1)]
    masscone = MassCone(1000, node_list, 3, 1/2, 2)
    [x1, y1, vx1, vy1] = masscone.predict_node(1/3, 1/3)
    [x2, y2, vx2, vy2] = masscone.predict_node(1/3, 2/3)
    [x3, y3, vx3, vy3] = masscone.predict_node(2/3, 1/3)
    [x4, y4, vx4, vy4] = masscone.predict_node(2/3, 2/3)
    [x0, y0, vx0, vy0] = masscone.predict_node(1/2, 1/2)
    plt.figure()
    x = [p.x for p in node_list]
    y = [p.y for p in node_list]
    vx = [p.vx for p in node_list]
    vy = [p.vy for p in node_list]
    plt.plot(x, y)
    plt.scatter([x1, x2, x3, x4], [y1, y2, y3, y4])
    plt.plot(vx, vy)
    plt.scatter(vx1, vy1)
    plt.scatter(x0, y0)
    plt.show()