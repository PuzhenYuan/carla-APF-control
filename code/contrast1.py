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
import evalfun

actor_list = []

class Node: #是存储路径时的结点，类似结构体
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.v = np.sqrt(np.power(self.vx, 2) + np.power(self.vy, 2))
        if np.abs(vx) <= 0.001:
            if vx >= 0:
                vx = 0.001
            else:
                vx = -0.001
        self.theta = np.arctan(vy/vx)

    def update(self): #用来更新参数v，theta
        self.v = np.sqrt(np.power(self.vx, 2) + np.power(self.vy, 2))
        vx = self.vx
        if np.abs(vx) <= 0.001:
            if vx >= 0:
                vx = 0.001
            else:
                vx = -0.001
        self.theta = np.arctan(self.vy/vx)

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

try:
    #初始化操作
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    blue_print_library = world.get_blueprint_library()
    car_bp = blue_print_library.filter("model3")[0]
    
    spawn_point0 = Transform(Location(x=65, y=25, z=3), Rotation(pitch=0, yaw=180, roll=0))
    spawn_point1 = Transform(Location(x=50, y=28.5, z=3), Rotation(pitch=0, yaw=180, roll=0))
    spawn_point2 = Transform(Location(x=30, y=25, z=3), Rotation(pitch=0, yaw=180, roll=0))
    
    mycar = world.spawn_actor(car_bp, spawn_point0)
    car1 = world.spawn_actor(car_bp, spawn_point1)
    car2 = world.spawn_actor(car_bp, spawn_point2)

    #几个车的质量，采用接近真实的数据，单位kg，方便迁移
    mycarm = 1000
    car1m = 1000
    car2m = 1000

    actor_list.append(mycar)
    actor_list.append(car1)
    actor_list.append(car2)
    
    #目标点target(x=35, y=28.5)，可以有别的设计
    #意图：本文件的对应工况是避险换道，因此设计目标点在另外一条车道，这是由行为决策做出的判断，不由本代码实现
    targetx = 35
    targety = 28.5
    
    #道路的y坐标
    roady1 = 22.5
    roady2 = 31.5
    
    time.sleep(2)
    
    pass #以后改掉，这边也可以初始化car1 car2速度，先写一个静止的版本
    mycar.apply_control(carla.VehicleControl(throttle=0.7, steer=0))
    time.sleep(1)
    
    node_num = 10 #每条路径规划的点数
    localtime = 0
    
    #对v和theta分别建立pid控制系统
    pid_v = PID(0.05, 0, 0)
    pid_theta = PID(5, 0, 0.1)
    
    #用于画图前的数据存储
    pathx_list = []
    pathy_list = []
    pathx_model = []
    pathy_model = []
    carx_list = []
    cary_list = []
    car1x_list = []
    car1y_list = []
    car1vx_list = []
    car1vy_list = []
    car2x_list = []
    car2y_list = []
    car2vx_list = []
    car2vy_list = []
    targetx_list = []
    
    while True:
        
        time_b = time.time()
        
        #生成三随机数
        r1 = random.random()
        r2 = random.random()
        r3 = random.random()
        
        rate_car = r1 / (r1 + r2 + r3) #标签概率，对car2采用
        rate_man = r2 / (r1 + r2 + r3)
        rate_bike = r3 / (r1 + r2 + r3)
        
        # rate_car = r1 / (r1 + r2)
        # rate_man = r2 / (r1 + r2)
        rate_bike = 0.1
        
        if rate_car >= rate_man and rate_car >= rate_bike:
            rate_car = 1
            rate_man = 0
            rate_bike = 0
        elif rate_man >= rate_car and rate_man >= rate_bike:
            rate_car = 0
            rate_man = 1
            rate_bike = 0
        else:
            rate_car = 0  
            rate_man = 0
            rate_bike = 1
        
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
        
        t = 0.1 #不同节点间的时间
        if mycarx - targetx <= 30: #移动目标点
            targetx = targetx - abs(mycarvx) * t
        
        #计算场力，考虑多路径
        routes_list = []
        route_model = []
        ratio_list = [0.5, 1, 1.5, 2]
        
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
            
                #计算每时每刻合力
                car1_potential_field = field.CarPotentialField(car1m, car1x, car1y, car1vx, car1vy)
                [f1x, f1y] = car1_potential_field.get_force(carx, cary, carvx, carvy)
                
                car2_potential_field = field.CarPotentialField(car2m, car2x, car2y, car2vx, car2vy)
                [f2x, f2y] = car2_potential_field.get_force(carx, cary, carvx, carvy)
                
                human_potential_field = field.HumanPotentialField(car2x, car2y, car2vx, car2vy)
                [fmx, fmy] = human_potential_field.get_force(carx, cary, carvx, carvy)
                
                target_field = field.TargetField(targetx, targety, ratio)
                [ftx, fty] = target_field.get_force(carx, cary, carvx, carvy)
                
                road_field1 = field.RoadPotentialField(roady1)
                fry1 = road_field1.get_force(carx, cary)
                road_field2 = field.RoadPotentialField(roady2)
                fry2 = road_field2.get_force(carx, cary) #可以写在一块函数中
                
                fx = f1x + rate_car * f2x + rate_man * fmx + ftx #两个系数是比例，手动调节，验证是否更加安全，欲采用Dse指标，见论文
                fy = f1y + rate_car * f2y + rate_man * fmy + fty + fry1 + fry2
                if (fx**2 + fy**2)**(1/2) <= 2000: #若陷入势能盆地则加入高斯模糊
                    fx = fx + np.random.normal(loc=0, scale=abs(ftx)/5)
                    fy = fy + np.random.normal(loc=0, scale=abs(fty)/5)
                
                #规划下一阶段结点，添加到route中，注意深拷贝，有发散风险，非真实(非MPC)
                #本质是将车辆质点化，个人认为这只是APF语境下、有限算力的制约下、在MPC的思想指导下，的一种较方便和粗糙的近似
                #系数经过配凑，勿轻易更改，之后可以微调比重，但是量的大小大致别动了，过程不严谨，仅仅好用
                node = copy.deepcopy(route[i-1])
                node.vx = 0.4 * node.vx + 4 * fx * t / mycarm
                node.vy = 0.4 * node.vy + 4 * fy * t / mycarm
                node.x = node.x + node.vx * t
                node.y = node.y + node.vy * t
                node.update()
                route.append(node)
                
                #更新参数
                carx = node.x
                cary = node.y
                carvx = node.vx
                carvy = node.vy
            
            #添加到路径集中
            routes_list.append(route)
        
        #选择最优路径
        [path, crash] = evalfun.get_best(routes_list)
        if crash: #会撞
            throttle = 0
            steer = 0
            brake = 1
        
        else: #不会撞
            #画图辅助
            if localtime % 20 == 0:
                pathx = copy.deepcopy(pathx_model)
                pathy = copy.deepcopy(pathy_model)
                for j in range(0, node_num):
                    pathx.append(path[j].x)
                    pathy.append(-path[j].y)
                pathx_list.append(pathx)
                pathy_list.append(pathy)
                carx_list.append(mycarx)
                cary_list.append(-mycary)
                car1x_list.append(car1x)
                car1y_list.append(-car1y)
                car1vx_list.append(car1vx)
                car1vy_list.append(-car1vy)
                car2x_list.append(car2x)
                car2y_list.append(-car2y)
                car2vx_list.append(car2vx)
                car2vy_list.append(-car2vy)
                targetx_list.append(targetx)
            
            #采用PID控制
            #可以认为node[9]为theta预瞄点，不考虑后续的点，也可以用b样条拟合后再取点(这边采用)
            #v的控制采用定速巡航
            print('------------- virtual velocity ------------')
            print('node0_v: ' + str(path[0].v))
            print('node1_v: ' + str(path[1].v))
            print('node2_v: ' + str(path[2].v))
            print('node3_v: ' + str(path[3].v))
            print('node4_v: ' + str(path[4].v))
            
            pid_v.update_e(15 - path[0].v)
            u_v = pid_v.get_u()
            
            #第二个参数取值0~1，越接近1则预瞄越远，预瞄角度选取是由预瞄点的方位角与切线角组成
            target_theta = bspline.b_azimuth(path, 0.6)*(3/6) + bspline.b_tangent(path, 0.6)*(3/6)
            pid_theta.update_e(target_theta - path[0].theta)
            u_theta = pid_theta.get_u()
            
            throttle = u_v
            steer = u_theta
            brake = 0
        
        #控制车辆按轨迹行驶
        mycar.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))
        car1.apply_control(carla.VehicleControl(throttle=0.3, steer=0))  #throttle=0~0.3
        car2.apply_control(carla.VehicleControl(throttle=0, steer=0))  #throttle>=0
        
        # if crash:
        #     time.sleep(4)
        
        localtime = localtime + 1
        if localtime > 1700: #退出机制
            print('break because of limited localtime')
            break
        
        time_e = time.time()
        print('time cost in total: ' + str(time_e - time_b))



















    #制作动画
    fig = plt.figure(figsize=(12, 4))
    def basis(i):
        if i >= len(pathx_list):
            i = len(pathx_list)-1
        if i == 0:
            time.sleep(1)
        plt.clf()
        pathi = []
        pathx = pathx_list[i]
        pathy = pathy_list[i]
        for j in range(0, len(pathx)):
            pathi.append(Node(pathx[j], pathy[j], 0, 0))
        xb = pathx[0]
        yb = pathy[0]
        car1_x = car1x_list[i]
        car1_y = car1y_list[i]
        car1_vx = car1vx_list[i]
        car1_vy = car1vy_list[i]
        car2_x = car2x_list[i]
        car2_y = car2y_list[i]
        car2_vx = car2vx_list[i]
        car2_vy = car2vy_list[i]
        targetx = targetx_list[i]
        
        #画第一张图
        
        rect1 = [0.07, 0.12, 0.28, 0.8] #左下宽高
        ax1 = plt.axes(rect1)
        ax1.axis([-10+xb, 10+xb, -10+yb, 10+yb])
        ax1.set_xlabel('x-position')
        ax1.set_ylabel('y-position')
        ax1.set_title('local map')
        
        x = np.linspace(-10+xb, 10+xb, num=50)
        y = np.linspace(-10+yb, 10+yb, num=50)
        [X, Y] = np.meshgrid(x, y)
        p1 = field.CarPotentialField(car1m, car1_x, car1_y, car1_vx, car1_vy).get_potential(X, Y, 0, 0)
        p2 = field.CarPotentialField(car2m, car2_x, car2_y, car2_vx, car2_vy).get_potential(X, Y, 0, 0) * rate_car
        pt = field.TargetField(targetx, -targety, 1).get_potential(X, Y)
        pr = field.RoadPotentialField(-roady1).get_potential(X, Y) + field.RoadPotentialField(-roady2).get_potential(X, Y)
        pm = field.HumanPotentialField(car2_x, car2_y, car2_vx, car2_vy).get_potential(X, Y, 0, 0) * rate_man
        
        print('--------------- position potential ---------------')
        print('p1: ' + str(p1[25][25]))
        print('p2: ' + str(p2[25][25]))
        print('pt: ' + str(pt[25][25]))
        print('pr: ' + str(pr[25][25]))
        print('pm: ' + str(pm[25][25]))
        print('sum: ' + str((p1 + p2 + pt + pr + pm)[25][25]))
        Z = np.minimum(p1 + p2 + pt + pr + pm, 1000000)
        ctf = ax1.contourf(X, Y, Z, levels=np.linspace(0, 1000000, 30))
        # ax1.clabel(ctf, inline=True, fontsize=10)
        
        ax1.scatter(xb, yb, color='green', linewidths=25)
        ax1.plot(carx_list[0:i], cary_list[0:i], color='green')
        ax1.scatter(pathx, pathy, color='orange')
        # ax1.plot(pathx, pathy, color='orange')
        
        ax1.scatter(car1_x, car1_y, color='red', linewidths=25)
        # ax1.plot(car1x_list[0:i], car1y_list[0:i], color='red')
        ax1.scatter(car2_x, car2_y, color='red', linewidths=25)
        # ax1.plot(car2x_list[0:i], car2y_list[0:i], color='red')
        
        X1 = []
        Y1 = []
        for t in np.linspace(0, 1, 50):
            [x1, y1] = bspline.bezier(pathi, t)
            X1.append(x1)
            Y1.append(y1)
        plt.plot(X1, Y1, color='red')
        
        #画第二张图
        
        rect2 = [0.41, 0.32, 0.54, 0.4] #左下宽高
        ax2 = plt.axes(rect2)
        ax2.axis([0, 70, -35, -20])
        ax2.set_xlabel('x-position')
        ax2.set_ylabel('y-position')
        ax2.set_title('general map')
        
        x = np.linspace(0, 70, num=50)
        y = np.linspace(-35, -20, num=50)
        [X, Y] = np.meshgrid(x, y)
        p1 = field.CarPotentialField(car1m, car1_x, car1_y, car1_vx, car1_vy).get_potential(X, Y, 0, 0)
        p2 = field.CarPotentialField(car2m, car2_x, car2_y, car2_vx, car2_vy).get_potential(X, Y, 0, 0) * rate_car
        pt = field.TargetField(targetx, -targety, 1).get_potential(X, Y)
        pr = field.RoadPotentialField(-roady1).get_potential(X, Y) + field.RoadPotentialField(-roady2).get_potential(X, Y)
        pm = field.HumanPotentialField(car2_x, car2_y, car2_vx, car2_vy).get_potential(X, Y, 0, 0) * rate_man
        Z = np.minimum(p1 + p2 + pt + pr + pm, 1000000)
        ctf = ax2.contourf(X, Y, Z, levels=np.linspace(0, 1000000, 30))
        # ax1.clabel(ctf, inline=True, fontsize=10)
        
        ax2.scatter(xb, yb, color='green', linewidths=15)
        ax2.plot(carx_list[0:i], cary_list[0:i], color='green')
        ax2.scatter(pathx, pathy, color='orange', linewidths=1)
        # ax2.plot(pathx, pathy, color='orange')
        
        ax2.scatter(car1_x, car1_y, color='red', linewidths=15)
        # ax2.plot(car1x_list[0:i], car1y_list[0:i], color='red')
        ax2.scatter(car2_x, car2_y, color='red', linewidths=15)
        # ax2.plot(car2x_list[0:i], car2y_list[0:i], color='red')
        
        X1 = []
        Y1 = []
        for t in np.linspace(0, 1, 50):
            [x1, y1] = bspline.bezier(pathi, t)
            X1.append(x1)
            Y1.append(y1)
        plt.plot(X1, Y1, color='red')
        
    animator = ani.FuncAnimation(fig, basis, interval=10)
    plt.show()



finally:
    for actor in actor_list:
        actor.destroy()
    print("all cleaned up!")