import math
import time

def get_best(routes_list): #路径选择
    hold = 0
    all_will_crash = True
    best_route = routes_list[0]
    for route in routes_list:
        mark, will_crash = evaluate(route)
        if mark >= hold and will_crash == False:
            best_route = route
            hold = mark
            all_will_crash = False
        elif will_crash == False:
            all_will_crash = False
        else:
            pass
    return best_route, all_will_crash

def evaluate(route, alpha_1=0.25, alpha_2=0.25, alpha_3=0.25, alpha_4=0.25, dt=0.1): #评价函数
    
    #初始数据积累与储存
    node_num = len(route)
    # node_num表示列表的长度，也就是分点的个数，设有n个分点
    dx = []
    # dx表示x坐标位移变化量，n-1个
    dy = []
    # dy表示y坐标位移变化量，n-1个
    dr = []
    # dr表示位置变化大小，n-1个
    vx = []
    # vx表示x分量速率，n-1个
    vy = []
    # vy表示y分量速率，n-1个
    v = []
    #v表示速率，n-1个
    ax = []
    # ax表示x分量加速率，n-2个
    ay = []
    # ay表示y分量加速率，n-2个
    a = []
    # a表示加速度大小，n-2个
    k = []
    # k表示曲率，n-2个
    
    for i in range(0, node_num - 1):
        dx.append(route[i + 1].x - route[i].x)
        dy.append(route[i + 1].y - route[i].y)
    dr = [(dx_**2 + dy_**2)**(1/2) for dx_, dy_ in zip(dx, dy)]
    vx = [dx_/dt for dx_ in dx]
    vy = [dy_/dt for dy_ in dy]
    v = [(vx_**2 + vy_**2)**(1/2) for vx_, vy_ in zip(vx, vy)]
    for i in range(0, node_num - 2):
        ax.append((vx[i + 1] - vx[i]) / dt)
        ay.append((vy[i + 1] - vy[i]) / dt)
    # a = [math.sqrt(ax_**2 + ay_**2) for ax_, ay_ in zip(ax, ay)]
    for i in range(0, node_num - 2):
        k.append((abs(vx[i] * ay[i] - vy[i] * ax[i])) / (vx[i]**2 + vy[i]**2 + 0.001)**(3/2))

    #判断是否撞车
    will_crash = False
    
    #路径不能存在折返现象
    for i in range(0, node_num - 2):
        dot_product = dx[i] * dx[i+1] + dy[i] * dy[i+1]
        if dot_product <= 0:
            will_crash = True
    
    #路径不能因为发散而太长或太短
    deltax = route[-1].x - route[0].x
    deltay = route[-1].y - route[0].y
    deltar = (deltay**2 + deltax**2)**(1/2)
    if  deltar <= 9 or deltar >= 20:
        will_crash = True
    
    #路径上梯度变化不能过大，保证平滑性
    for i in range(0, node_num - 2):
        rate = dr[i] / (dr[i+1] + 0.001)
        if rate >= 4 or rate <= 1/4:
            will_crash = True
    
    # arc_length = sum([(x**2 + y**2)**(1/2) for x, y in zip(dx, dy)])
    # if arc_length <= 10:
    #     will_crash = True

    #如果发生撞车
    if will_crash == True:
        mark = -100000
        return mark, will_crash

    #如果没有撞车
    else:
        beta_1 = 0
        for x in k:
            beta_1 += x
        beta_2 = 0
        for i in range(0, node_num - 2):
            beta_2 += math.fabs(v[i + 1] - v[i])
        beta_3 = 0
        for x in k:
            if x > beta_3:
                beta_3 = x
            else:
                pass
        beta_4 = 0
        for x in v:
            beta_4 += x
        mark = alpha_1 * beta_1 + alpha_2 * beta_2 + alpha_3 * beta_3 + alpha_4 * beta_4
        return mark, will_crash

# 测试代码
if __name__ == '__main__':

    class Node:  # 是存储路径时的结点，类似结构体
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
            self.theta = math.atan(vy / vx)

        def update(self):  # 用来更新参数v，theta
            self.v = math.sqrt(math.pow(self.vx, 2) + math.pow(self.vy, 2))
            vx = self.vx
            if abs(vx) <= 0.001:
                if vx >= 0:
                    vx = 0.001
                else:
                    vx = -0.001
            self.theta = math.atan(self.vy / vx)


    route = [Node(1, 1, 1, 1), Node(2, 2, 1, 1), Node(3, 3, 1, 1), Node(4, 4, 1, 1)]
    alpha_1 = 0.25
    alpha_2 = 0.25
    alpha_3 = 0.25
    alpha_4 = 0.25
    dt = 0.1
    tic = time.time()
    mark, will_crush = evaluate(route,alpha_1,alpha_2,alpha_3,alpha_4,1,1,1,dt)
    toc = time.time()
    if will_crush:
        print("Crush!")
        print('time: ' + str(toc - tic))
    else:
        print(mark)
        print('time: ' + str(toc - tic))