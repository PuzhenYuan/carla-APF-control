import sys
import glob
import os
import math
from matplotlib import pyplot as plt

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

from carla import Transform,Location,Rotation
import numpy as np
import time
import random

actor_list = [] #actor的列表

try:
    client = carla.Client("localhost",2000) #建立客户端
    client.set_timeout(5.0) #超时设定
    world = client.get_world() #获取、生成世界
    blueprint_library = world.get_blueprint_library() #获得与世界可以交互的蓝图
    car_bp = blueprint_library.filter("model3")[0] #从库中筛选对应蓝图，可以看看接口
    
    spawn_point1 = Transform(Location(x=60, y=25.5, z=5), Rotation(pitch=0, yaw=180, roll=0))
    spawn_point2 = Transform(Location(x=40, y=30, z=10), Rotation(pitch=0, yaw=180, roll=0))
    mycar = world.spawn_actor(car_bp, spawn_point1)
    car2 = world.spawn_actor(car_bp, spawn_point2)
    actor_list.append(mycar)
    actor_list.append(car2)
    
    time.sleep(2)
    
    throttle = 0.35
    steer = -0.01
    mycar.apply_control(carla.VehicleControl(throttle=throttle, steer=steer)) 
    
    x = []
    y = []
    vx = []
    vy = []
    v = []
    t = []
    localtime = 0
    tic = time.time()
    while True:
        mycarx = mycar.get_location().x
        mycary = mycar.get_location().y
        mycarvx = mycar.get_velocity().x
        mycarvy = mycar.get_velocity().y
        print('------------------------------------')
        print('mycarx: ' + str(mycarx))
        print('mycary: ' + str(mycary))
        print('mycarvx: ' + str(mycarvx))
        print('mycarvy: ' + str(mycarvy))
        mycar.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))
        
        toc = time.time()
        x.append(mycarx)
        y.append(-mycary)
        vx.append(mycarvx)
        vy.append(mycarvy)
        mycarv = math.sqrt(math.pow(mycarvx, 2) + math.pow(mycarvy, 2))
        v.append(mycarv)
        t.append(toc-tic)
        
        # localtime = localtime + 1
        # if localtime > 10000: #退出机制
        #     print('break because of limited localtime')
        #     break
        
        if toc - tic >= 5:
            throttle = 0
            
        
        if toc - tic >= 10:
            print('break because of limited time')
            
            fig = plt.figure()
            ax1 = fig.add_subplot(2, 2, 1)
            ax1.plot(t, x)
            ax1.set_title('t-x')
            ax2 = fig.add_subplot(2, 2, 2)
            ax2.plot(t, vx)
            ax2.set_title('t-vx')
            ax3 = fig.add_subplot(2, 2, 3)
            ax3.plot(x, y)
            ax3.set_title('x-y')
            ax4 = fig.add_subplot(2, 2, 4)
            ax4.plot(t, v)
            ax4.set_title('t-v')
            fig.show()
            input()
            break

finally:
    for actor in actor_list:
        actor.destroy()
    print("all cleaned up!")