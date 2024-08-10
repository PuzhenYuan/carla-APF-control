import math
import numpy as np
from matplotlib import cm
from matplotlib import pyplot as plt

class CarPotentialField: #可以简化类的设计，直接传递车辆本身作为参数，压缩代码量，但速度会变慢
    def __init__(self, carm, carx, cary, carvx, carvy):
        #def __init__(self, car, carm)
        #构造他车于(x,y)处产生风险场
        self.m = carm
        self.x = carx
        self.y = cary
        self.vx = carvx
        self.vy = carvy
        self.v = np.sqrt(np.power(self.vx, 2) + np.power(self.vy, 2)) + 0.001
        self.k1 = 2 #r在分母上的指数
        self.k2 = 1 #exp中的系数
        self.k3 = 100 #最前面系数

    def get_potential(self, mycarx, mycary, mycarvx, mycarvy):
        r = np.sqrt(np.power(self.x - mycarx, 2) + np.power(self.y - mycary, 2))
        p = self.m * self.k3 * np.exp(self.k2 * (self.vx * (mycarx - self.x) + self.vy * (mycary - self.y)) / r) / np.power(r, self.k1)
        return p
    
    def get_force(self, mycarx, mycary, mycarvx, mycarvy): #改为不对指数部分求导
        r = np.sqrt(np.power(self.x - mycarx, 2) + np.power(self.y - mycary, 2))
        e = np.exp(self.k2 * (self.vx * (mycarx - self.x) + self.vy * (mycary - self.y)) / r)
        fx = self.k1 * self.k3 * self.m * e * (mycarx - self.x) / np.power(r, self.k1 + 2)
        fy = self.k1 * self.k3 * self.m * e * (mycary - self.y) / np.power(r, self.k1 + 2)
        return fx, fy

class TargetField:
    def __init__(self, targetx, targety, ratio):
        self.x = targetx
        self.y = targety
        self.ratio = ratio
    
    def get_potential(self, mycarx, mycary):
        k = self.ratio 
        x = mycarx - self.x
        y = mycary - self.y
        r = np.sqrt(np.power(x, 2) + np.power(y, 2))
        F = 10000
        p = k * F * r
        return p

    def get_force(self, mycarx, mycary, mycarvx, mycarvy):
        k = self.ratio 
        x = mycarx - self.x
        y = mycary - self.y
        r = np.sqrt(np.power(x, 2) + np.power(y, 2))
        F = 10000
        fx = -k * F * (x / r)
        fy = -k * F * (y / r)
        return fx, fy

class RoadPotentialField:
    def __init__(self, roady):
        self.y = roady
        self.k1 = 2
        self.m = 20000
    
    def get_potential(self, mycarx, mycary):
        y = mycary - self.y
        p = self.m / np.power(y, self.k1)
        return p
    
    def get_force(self, mycarx, mycary):
        y = mycary - self.y
        fx = 0
        fy = self.k1 * self.m / np.power(y, self.k1 + 1)
        return fy

class PointPotentialField:
    def __init__(self, pointx, pointy):
        self.x = pointx
        self.y = pointy
        self.m = 20000
        self.k1 = 2
    
    def get_potential(self, mycarx, mycary):
        x = mycarx - self.x
        y = mycary - self.y
        p = self.m / np.power((x**2 + y**2)**(1/2), self.k1)
        return p
    
    def get_force(self, mycarx, mycary):
        x = mycarx - self.x
        y = mycary - self.y
        r = (x**2 + y**2)**(1/2)
        fx = self.m * self.k1 * x / r**(self.k1 + 2)
        fy = self.m * self.k1 * y / r**(self.k1 + 2)
        return fx, fy

class HumanPotentialField:
    def __init__(self, manx, many, manvx, manvy):
        self.x = manx
        self.y = many
        self.vx = manvx
        self.vy = manvy
        self.v = np.sqrt(np.power(manvx, 2) + np.power(manvy, 2))
        if self.v <= 0.001:
            self.v = 0.001
            self.vx = -0.001
            self.vy = 0
        else:
            pass
        self.m = 1000
    
    def get_potential(self, mycarx, mycary, mycarvx, mycarvy):
        k = 3 + self.v / 10
        r = 10
        a = r + self.v
        b = r
        x = mycarx - self.x
        y = mycary - self.y
        x1 = (self.vx * x + self.vy * y) / self.v
        y1 = (self.vy * x - self.vx * y) / self.v
        p = k * self.m / (x1**2 / a**2 + y1**2 / b**2)
        return p
        
    def get_force(self, mycarx, mycary, mycarvx, mycarvy):
        k = 1 + self.v / 10
        r = 10
        a = r + self.v
        b = r
        x = mycarx - self.x
        y = mycary - self.y
        x1 = (self.vx * x + self.vy * y) / self.v
        y1 = (self.vy * x - self.vx * y) / self.v
        fx = k * self.m * (x1**2 / a**2 + y1**2 / b**2)**(-2) * (2 * x1 * self.vx / (a**2 * self.v) + 2 * y1 * self.vy / (b**2 * self.v))
        fy = k * self.m * (x1**2 / a**2 + y1**2 / b**2)**(-2) * (2 * x1 * self.vy / (a**2 * self.v) - 2 * y1 * self.vx / (b**2 * self.v))
        return fx, fy
        

class BikePotentialField:
    def __init__(self, bikex, bikey, bikevx, bikevy):
        pass
    
    def get_potential(self, mycarx, mycary, mycarvx, mycarvy):
        pass
    
    def get_force(self, mycarx, mycary, mycarvx, mycarvy):
        pass

#测试代码
if __name__ == "__main__":
    
    plt.figure(figsize=(6, 6))
    x = np.arange(start=-5, stop=5, step=0.01)
    y = np.arange(start=-5, stop=5, step=0.01)
    [X, Y] = np.meshgrid(x, y)
    
    # p = CarPotentialField(1000, 0, 0, 0, 2).get_potential(X, Y, 0, 0)
    # Z = np.minimum(p, 100000)
    # [fx, fy] = CarPotentialField(1000, 0, 0, 0, 0).get_force(X, Y, 0, 0)
    # Z = np.minimum((fx**2 + fy**2)**(1/2), 10000)
    
    # p = RoadPotentialField(0).get_potential(X, Y)
    # Z = np.minimum(p, 100000)
    # fry = RoadPotentialField(0).get_force(X, Y)
    # Z = np.minimum(np.abs(fry), 10000)
    
    # p = HumanPotentialField(0, 0, 0, 2).get_potential(X, Y, 0, 0)
    # Z = np.minimum(p, 100000)
    # [fx, fy] = HumanPotentialField(0, 0, 0, 0).get_force(X, Y, 0, 0)
    # Z = np.minimum((fx**2 + fy**2)**(1/2), 10000)
    
    # p = PointPotentialField(0, 0).get_potential(X, Y)
    # Z = np.minimum(p, 100000)
    [fpx, fpy] = PointPotentialField(0, 0).get_force(X, Y)
    Z = np.minimum((fpx**2 + fpy**2)**(1/2), 10000)
    
    ax = plt.axes([0.1, 0.1, 0.8, 0.8])
    ax.contourf(X, Y, Z, cmap=cm.coolwarm)
    ax.set_xlabel('x-position')
    ax.set_ylabel('y-position')
    plt.show()