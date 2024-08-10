import sys
import glob
import os
import math
import copy
from matplotlib import pyplot as plt
from matplotlib import animation as ani

from carla import Transform, Location, Rotation
import numpy as np
import time
import random
import evalfun

try:
    sys.path.append(
        glob.glob(
            '../WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

#自编库
import field
import bspline
import masscone

actor_list = []

class Node: #是存储路径时的结点，类似结构体
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.v = math.sqrt(math.pow(self.vx, 2) + math.pow(self.vy, 2))
        if abs(vx) <= 0.001:
            if vx >= 0:
                vx = 0.001
            else:
                vx = -0.001
        self.theta = math.atan(vy/vx)

    def update(self): #用来更新参数v，theta
        self.v = math.sqrt(math.pow(self.vx, 2) + math.pow(self.vy, 2))
        vx = self.vx
        if abs(vx) <= 0.001:
            if vx >= 0:
                vx = 0.001
            else:
                vx = -0.001
        self.theta = math.atan(self.vy/vx)

class PID: #pid算法对应的类，实例是对一个量的跟踪控制系统
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ep = 0
        self.ei = 0
        self.ed = 0
        self.t = 0.1

    def update_e(self, e): #获取新误差e后调用此函数更新ep，ei，ed
        # print(e)
        self.ed = e - self.ep
        self.ei += e
        self.ep = e

    def get_u(self): #返回控制量u
        u = self.kp * self.ep + self.ki * self.ei + self.kd * self.ed
        return u

def get_best(routes_list): #路径选择
    hold = 0
    q = 1
    s = 1
    c = 1
    e = 1
    for route in routes_list:
        mark = evalfun.evaluate(route, q, s, c, e)
        if mark >= hold:
            best_route = route
            hold = mark
        else:
            pass
    return best_route

try:
    #初始化操作
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    blue_print_library = world.get_blueprint_library()
    car_bp = blue_print_library.filter("model3")[0]
    
    spawn_point1 = Transform(Location(x=65, y=25, z=5), Rotation(pitch=0, yaw=180, roll=0))
    spawn_point2 = Transform(Location(x=50, y=28, z=5), Rotation(pitch=0, yaw=180, roll=0))
    spawn_point3 = Transform(Location(x=30, y=25, z=5), Rotation(pitch=0, yaw=180, roll=0))
    
    mycar = world.spawn_actor(car_bp, spawn_point1)
    car1 = world.spawn_actor(car_bp, spawn_point2)
    car2 = world.spawn_actor(car_bp, spawn_point3)

    #几个车的质量，采用接近真实的数据，单位kg，方便迁移
    mycarm = 1000 #无所谓
    car1m = 1000
    car2m = 1000

    actor_list.append(mycar)
    actor_list.append(car1)
    actor_list.append(car2)
    
    #目标点 target(x=10, y=28.5)，可以有别的设计
    targetx = 10
    targety = 28.5
    
    time.sleep(2)
    
    pass #以后改掉，这边可以初始化car1 car2速度，先写一个静止的版本
    mycar.apply_control(carla.VehicleControl(throttle=0.7, steer=0))
    time.sleep(1)
    
    node_num = 10 #每条路径规划的点数
    localtime = 0
    
    #对v和theta分别建立pid控制系统
    pid_v = PID(0.05, 0, 0)
    pid_theta = PID(5, 0, 0.1)

    car1x = car1.get_location().x
    car1y = car1.get_location().y
    car1vx = car1.get_velocity().x
    car1vy = car1.get_velocity().y
    nodes_1 = [Node(car1x, car1y, car1vx, car1vy) for i in range(0, 5)]
    
    car2x = car2.get_location().x
    car2y = car2.get_location().y
    car2vx = car2.get_velocity().x
    car2vy = car2.get_velocity().y
    nodes_2 = [Node(car2x, car2y, car2vx, car2vy) for i in range(0, 5)]
    
    while True:
        
        time_b = time.time()
        
        #获取信息
        mycarx = mycar.get_location().x
        mycary = mycar.get_location().y
        mycarvx = mycar.get_velocity().x
        mycarvy = mycar.get_velocity().y
        
        car1x = car1.get_location().x
        car1y = car1.get_location().y
        car1vx = car1.get_velocity().x
        car1vy = car1.get_velocity().y
        
        car2x = car2.get_location().x
        car2y = car2.get_location().y
        car2vx = car2.get_velocity().x
        car2vy = car2.get_velocity().y
        
        #更新质量锥
        nodes_1.insert(0, Node(car1x, car1y, car1vx, car1vy))
        nodes_1.pop()
        masscone_1 = masscone.MassCone(car1m, nodes_1, 3, 0.5, 2)
        
        nodes_2.insert(0, Node(car2x, car2y, car2vx, car2vy))
        nodes_2.pop()
        masscone_2 = masscone.MassCone(car2m, nodes_2, 3, 0.5, 2)
        
        #计算场力，考虑多路径
        routes_list = []
        route_model = []
        ratio_list = [0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2]
        t = 0.1
        
        time_5 = time.time()
        
        for ratio in ratio_list:
        
            #初始化循环条件
            route = copy.deepcopy(route_model)
            carx = mycarx
            cary = mycary
            carvx = mycarvx
            carvy = mycarvy
            
            node_b = Node(carx, cary, carvx, carvy)
            route.append(node_b)
            
            #生成单条路径
            for i in range(1, node_num): #已有0，规划1~num-1，共num个
            
                #计算每时每刻质量锥合力
                time_3 = time.time()
                [f1x, f1y] = masscone_1.get_field_force(carx, cary, carvx, carvy, 2, 2)
                [f2x, f2y] = masscone_2.get_field_force(carx, cary, carvx, carvy, 2, 2)
                time_4 = time.time()
                target_field = field.TargetField(targetx, targety, ratio)
                [ftx, fty] = target_field.get_force(carx, cary, carvx, carvy)
                
                fx = f1x + f2x + ftx
                fy = f1y + f2y + fty
                if (fx**2 + fy**2)**(1/2) <= 2000: #若陷入势能盆地则加入高斯模糊
                    fx = fx + np.random.normal(loc=0, scale=abs(ftx)/5)
                    fy = fy + np.random.normal(loc=0, scale=abs(fty)/5)
                
                #规划下一阶段结点，添加到route中，注意要深拷贝
                node = copy.deepcopy(route[i-1])
                node.vx = (1/4) * node.vx + (1/2) * fx * t #后面有发散风险，非真实，可能得引入归一化修正
                node.vy = (1/4) * node.vy + (1/2) * fy * t
                node.x = node.x + node.vx * t * (1/100)
                node.y = node.y + node.vy * t * (1/100)
                node.update()
                route.append(node)
                
                #更新参数
                carx = node.x
                cary = node.y
                carvx = node.vx
                carvy = node.vy
            
            #添加到路径集中
            routes_list.append(route)
        
        time_6 = time.time()
        
        #选择最优路径
        path = get_best(routes_list)
        
        #采用PID控制
        #可以认为node[9]为theta预瞄点，不考虑后续的点，也可以用b样条拟合后再取点(这边采用)
        #v的控制采用定速巡航
        pid_v.update_e(15 - path[0].v)
        u_v = pid_v.get_u()
        
        time_1 = time.time()
        target_theta = bspline.btheta(path, 0.8) #第二个参数取值0~1，越接近1则预瞄越远
        time_2 = time.time()
        
        pid_theta.update_e(target_theta - path[0].theta)
        #pid_theta.update_e(path[9].theta - path[0].theta)
        u_theta = pid_theta.get_u()
        
        throttle = u_v #注意这边不留参数
        steer = u_theta
        
        #控制车辆按轨迹行驶
        mycar.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))
        car1.apply_control(carla.VehicleControl(throttle=0, steer=0))  #throttle=0.25也行
        car2.apply_control(carla.VehicleControl(throttle=0, steer=0))  #throttle=0.15也行
        
        localtime = localtime + 1
        if localtime > 2000: #退出机制
            print('break because of limited localtime')
            break
        
        time_e = time.time()
        print('----------------------------------------')
        print('time cost in bspline: ' + str(time_2 - time_1))
        print('time cost in masscone: ' + str(time_4 - time_3))
        print('time cost in generating path: ' + str(time_6 - time_5))
        print('time cost in total: ' + str(time_e - time_b))

finally:
    for actor in actor_list:
        actor.destroy()
    print("all cleaned up!")