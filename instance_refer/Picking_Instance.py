'''
File: Picking_Instance.py
Project: Integrated_Picking---Sorting_Model
File Created: Saturday, 22nd April 2023 9:23:46 pm
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''

import sys
sys.path.append('..')
import numpy as np
import matplotlib.pyplot as plt
import Integrated_Picking_and_Sorting_Model.utils

class Map:
    """ map class
    Properties:
        block_width (int): 块的宽度
        block_length (int): 块的长度
        max_level (int): 货架的最大层数
        map_width (int): 地图的宽度
        map_length (int): 地图的长度
        idx_num (int): 地图中所有格子的个数
        idx2xy (dict): idx到xy的映射
        xy2idx (dict): xy到idx的映射
        types (list): 地图中格子的类型
        idx2type (dict): idx到type的映射
        type2idx (dict): type到idx的映射
    Methods:
        get_distance(idx1, idx2): 获取两位置之间的距离
        render(): 展示地图结构
    """
    def __init__(self, w_num, l_num):
        """__init__ generate map

        Args:
            w_num (int): 宽度方向上块的个数 (y方向)
            l_num (int): 长度方向上块的个数 (x方向)
        
        """
        # map size
        self.block_width = 2
        self.block_length = 18
        self.max_level = 10
        self.map_width = (self.block_width+1) * w_num + 1
        self.map_length = (self.block_length+1) * l_num + 1
        # set idx and xy
        self.idx_num = self.map_width * self.map_length
        self.idx2xy, self.xy2idx = {}, {}
        for idx in range(self.idx_num):
            x = idx % self.map_length # length direction
            y = idx // self.map_length # width direction
            self.idx2xy[idx] = (x, y)
            self.xy2idx[(x, y)] = idx
        # set types
        self.types = ["pod", "aisle", "pickerIn", "pickerOut"]
        self.idx2type = {}
        self.type2idx = {}
        for type in self.types:
            self.type2idx[type] = []
        for idx in range(self.idx_num):
            x, y = self.idx2xy[idx]
            if x == 0 and y == 0:
                # picker in
                self.idx2type[idx] = "pickerIn"
                self.type2idx["pickerIn"].append(idx)
            elif x == 0 and y == self.map_width - 1:
                # picker out
                self.idx2type[idx] = "pickerOut"
                self.type2idx["pickerOut"].append(idx)
            elif y % (self.block_width+1) == 0 or x % (self.block_length+1) == 0:
                # aisile
                self.idx2type[idx] = "aisle"
                self.type2idx["aisle"].append(idx)
            else:
                # pod
                self.idx2type[idx] = "pod"
                self.type2idx["pod"].append(idx)

    def get_distance(self, idx1, idx2, extra=True):
        if idx1 == idx2:
            return 0
        # 曼哈顿+分情况讨论额外距离 
        x1, y1 = self.idx2xy[idx1]
        x2, y2 = self.idx2xy[idx2]
        dist = manhattan_dis = abs(x1-x2) + abs(y1-y2)
        # add extra dist
        if extra:
            # 1. same block row extra dist
            if x1 // (self.block_length+1) == x2 // (self.block_length+1):
                if (y1 == y2 or (abs(y1-y2)==1 and (self.idx2type[idx1] == "aisle" or self.idx2type[idx2] == "aisle"))
                    or (abs(y1-y2)==2 and self.idx2type[self.xy2idx[x1, (y1+y2)//2]] == "aisle")):
                    # 豁免情况
                    pass
                else:
                    # 计算两个idx距离两边通道的距离
                    left1 = x1 % (self.block_length+1)
                    left2 = x2 % (self.block_length+1)
                    right1 = self.block_length - left1
                    right2 = self.block_length - left2
                    dist += 2 * min(left1, left2, right1, right2)
            # 2. aisle extra dist
            if y1 == y2:
                # same aisle
                dist += 2
            elif y1 > y2:
                # idx1 higher than idx2
                if self.idx2type[idx1] == "pod" and self.idx2type[self.xy2idx[x1, y1+1]] == "aisle": # aisle above idx1
                    dist += 2
                if self.idx2type[idx2] == "pod" and self.idx2type[self.xy2idx[x2, y2-1]] == "aisle": # aisle below idx2
                    dist += 2
            else:
                # idx1 lower than idx2
                if self.idx2type[idx1] == "pod" and self.idx2type[self.xy2idx[x1, y1-1]] == "aisle": # aisle below idx1
                    dist += 2
                if self.idx2type[idx2] == "pod" and self.idx2type[self.xy2idx[x2, y2+1]] == "aisle": # aisle above idx2
                    dist += 2
        return dist

    def render(self, routes=[]):
        ax = plt.gca()
        # draw map
        draw_tool = utils.DrawTools() 
        draw_tool.draw_map(ax, self)
        # draw routes
        for route in routes:
            for edge in route:
                # draw arrows
                x1, y1 = self.idx2xy[edge[0]]
                x2, y2 = self.idx2xy[edge[1]]
                plt.arrow(x1, y1, x2-x1, y2-y1, head_width=0.5, head_length=1, fc='k', ec='k')
        # show picture
        plt.show()
       

class Instance:
    """ instance class
    properties:
        tasks (List[Dict]) Dict["pos_idx", "level", "readyTime", "dueTime"]
        robots (List[Dict]) Dict["pos_idx"]
        nodes (List[Dict]) Dict["pos_idx", "level", "readyTime", "dueTime", "demand", "serviceTime"]
        disMatrix (np.ndarray[nodeNum, nodeNum])
        timeMatrix (np.ndarray[nodeNum, nodeNum])
    """
    def __init__(self, w_num, l_num, task_num, robot_num, seed=1, real_dist=True):
        """__init__ generate instance

        Args:
            w_num (int): 宽度方向上块的个数 (y方向)
            l_num (int): 长度方向上块的个数 (x方向)
            task_num (int): 任务的个数
            seed (int, optional): 随机种子. Defaults to 1.
            real_dist (bool, optional): 是否使用真实距离. Defaults to False.
        """
        # set params
        self.capacity = 8 # 机器人容量
        self.pick_time = 120 # 环形拣选台拣货时间
        self.min_time_gap = 10000 # readyTime和dueTime的最小间隔
        self.latest_ready_time = 100 # readyTime的最大值
        self.latest_due_time = 100000 # dueTime的最大值
        self.pack_time = 1 # 上下拣选台的时间
        self.lift_time = 1 # 升降货架的时间
        self.robot_speed = 1 # 机器人的速度
        self.real_dist = real_dist # 是否使用真实距离
        # set map
        self.map = Map(w_num, l_num)
        # set tasks
        self.taskNum = task_num
        self.tasks = self.generate_tasks(task_num, seed)
        # set robots
        self.robotNum = robot_num
        self.robots = self.generate_robots(robot_num)
        # get nodes
        self.n = task_num # 任务数
        self.nodeNum = 4 * task_num + robot_num # 节点数
        self.nodes = self.generate_nodes() 
        # split node set
        self.P1 = list(range(0, self.n)) # 任务起点
        self.P2 = list(range(self.n, 2*self.n)) # 任务回程起点
        self.D1 = list(range(2*self.n, 3*self.n)) # 任务终点
        self.D2 = list(range(3*self.n, 4*self.n)) # 任务回程终点
        self.W = list(range(4*self.n, 4*self.n+self.robotNum)) # 机器人起点
        self.N = self.P1 + self.P2 + self.D1 + self.D2 + self.W # 所有节点
        self.K = list(range(self.robotNum)) # 机器人编号
        # calculate distance/time matrix
        self.disMatrix = self.cal_disMatrix()
        self.timeMatrix = self.disMatrix / self.robot_speed
                
    def generate_tasks(self, task_num, seed):
        """generate_tasks 生成任务

        Args:
            task_num (int): 任务的个数
            seed (int): 随机种子

        Returns:
            list: 任务列表 (pod_idx, pod_level(from 1-10), readyTime, dueTime)
        """
        np.random.seed(seed)
        tasks = []
        tabu = {}
        for i in range(task_num):
            # generate task
            pos_idx = np.random.choice(self.map.type2idx["pod"])
            level = np.random.randint(1, self.map.max_level+1)
            # prevent generate task in same pos same level
            while (pos_idx, level) in tabu:
                pos_idx = np.random.choice(self.map.type2idx["pod"])
                level = np.random.randint(1, self.map.max_level+1)
            tabu[pos_idx, level] = 1
            readyTime = np.random.randint(0, self.latest_ready_time)
            dueTime = np.random.randint(readyTime+self.min_time_gap, self.latest_due_time)
            # add task
            task = {
                "pos_idx" : pos_idx, # 位置编号
                "level" : level, # 层数
                "readyTime" : readyTime, # 开始服务时间
                "dueTime" : dueTime, # 结束服务时间
            }
            tasks.append(task)
        return tasks
    
    def generate_robots(self, robot_num):
        """generate_robots 生成机器人位置

        Args:
            robot_num (int): 机器人数量
        """
        robots = []
        for i in range(robot_num):
            x, y = i+1, self.map.map_width-1 # 地图左下角
            idx = self.map.xy2idx[x, y]
            robot = {
                "pos_idx" : idx, # 位置编号
            }
            robots.append(robot)
        return robots
        
    def generate_nodes(self):
        """generate_nodes 生成节点

        5类节点: P1, P2, D1, D2, W, 其序号P1 < P2 < D1 < D2 < W
        节点属性: type, pos_idx, level, readyTime, dueTime, demand, serviceTime
            pos_idx = tasks[i]["pos_idx"] for all
            level = tasks[i]["level"] for all
            readyTime = tasks[i]["readyTime"] for D1; readyTime = 0 for others
            dueTime = tasks[i]["dueTime"] for D1; dueTime = latest_due_time for others
            demand = 1 for P1, P2; demand = -1 for D1, D2; demand = 0 for W
            serviceTime = level for P1, D1; serviceTime = 1 for P2, D2; serviceTime = 0 for W
        """
        self.node2type = {}
        nodes = []
        for i in range(self.taskNum):
            # P1
            node = {
                "type": "P1",
                "pos_idx": self.tasks[i]["pos_idx"],
                "level": self.tasks[i]["level"],
                "readyTime": self.tasks[i]["readyTime"],
                "dueTime": self.tasks[i]["dueTime"],
                "demand": 1,
                "serviceTime": self.tasks[i]["level"] * self.lift_time,
            }
            self.node2type[len(nodes)] = "P1"
            nodes.append(node)
        for i in range(self.taskNum):
            # P2
            node = {
                "type": "P2",
                "pos_idx": self.map.type2idx["pickerOut"][0],
                "level": self.tasks[i]["level"],
                "readyTime": 0,
                "dueTime": self.latest_due_time,
                "demand": 1,
                "serviceTime": self.pack_time,
            }
            self.node2type[len(nodes)] = "P2"
            nodes.append(node)
        for i in range(self.taskNum):
            # D1
            node = {
                "type": "D1",
                "pos_idx": self.map.type2idx["pickerIn"][0],
                "level": self.tasks[i]["level"],
                "readyTime": 0,
                "dueTime": self.tasks[i]["dueTime"],
                "demand": -1,
                "serviceTime": self.pack_time,
            }
            self.node2type[len(nodes)] = "D1"
            nodes.append(node)
        for i in range(self.taskNum):
            # D2
            node = {
                "type": "D2",
                "pos_idx": self.tasks[i]["pos_idx"],
                "level": self.tasks[i]["level"],
                "readyTime": 0,
                "dueTime": self.latest_due_time,
                "demand": -1,
                "serviceTime": self.tasks[i]["level"] * self.lift_time,
            }
            self.node2type[len(nodes)] = "D2"
            nodes.append(node)
        for i in range(self.robotNum):
            # W
            node = {
                "type": "W",
                "pos_idx": self.robots[i]["pos_idx"],
                "level": 0,
                "readyTime": 0,
                "dueTime": self.latest_due_time,
                "demand": 0,
                "serviceTime": 0,
            }
            self.node2type[len(nodes)] = "W"
            nodes.append(node)
        return nodes

    def cal_disMatrix(self):
        disMatrix = np.zeros((self.nodeNum, self.nodeNum))
        for i in range(self.nodeNum):
            for j in range(self.nodeNum):
                disMatrix[i, j] = self.map.get_distance(self.nodes[i]["pos_idx"], self.nodes[j]["pos_idx"], extra=self.real_dist)
        # self.check_disMatrix(disMatrix)
        return disMatrix

    def check_disMatrix(self, disMatrix):
        # 检查距离矩阵是否满足三角不等式
        wrong_pairs = [(i, k, j) for i in self.N for j in self.N for k in self.N if i!=j and j!=k and i!=k and disMatrix[i, j] > disMatrix[i, k] + disMatrix[k, j]]
        if len(wrong_pairs) > 0:
            print("disMatrix wrong")

    def render(self, routes=[], model=None):
        """
        input routes or model, draw map, robots, routes
        ps: routes here is instance routes
        """
        ax = plt.gca()
        draw_tool = utils.DrawTools() 
        # draw map
        draw_tool.draw_instance(ax, self)
        # draw routes
        if model is not None:
            routes = utils.model2instance_routes(model, self)
        if routes:
            map_routes = utils.instance_routes2map_routes(self, routes)
            draw_tool.draw_routes(ax, self.map, map_routes)
        # draw robots
        plt.show()

if __name__ == "__main__":
    # generate instance
    w_num = 3
    l_num = 3
    task_num = 20
    robot_num = 10
    instance = Instance(w_num, l_num, task_num, robot_num)
    print("generate {} tasks".format(10))
    # show structure
    instance.render()