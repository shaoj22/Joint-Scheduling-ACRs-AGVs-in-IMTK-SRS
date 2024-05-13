'''
File: integrated_gurobi_model_update.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a new update model for the picking and sorting model.
----------
Author: 626
Created Date: 2023.12.08
'''


import sys
sys.path.append('..')
import numpy as np
import gurobipy as gp
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance
from gurobipy import GRB


class IntegratedGurobiModel:
    def __init__(self, integrated_instance, time_limit=None, init_flag=True):
        """
        init the IntegratedGurobiModel with inputting instance.

        Args:
        integrated_instance (class): instance class of the problem. 
        """
        # common param
        self.integrated_instance = integrated_instance
        self.time_limit = time_limit
        self.bigM = 1200
        # picking param
        self.init_flag = init_flag
        self.Q = integrated_instance.capacity
        self.delta_T = integrated_instance.pick_time
        self.N = list(range(integrated_instance.nodeNum))
        self.nodes = integrated_instance.nodes
        self.n = integrated_instance.n
        self.P1 = integrated_instance.P1
        self.P2 = integrated_instance.P2
        self.D1 = integrated_instance.D1
        self.D2 = integrated_instance.D2
        self.W = integrated_instance.W
        self.K = list(range(integrated_instance.robotNum))
        self.disMatrix = integrated_instance.disMatrix
        self.timeMatrix = integrated_instance.timeMatrix

        # sorting param
        self.P = integrated_instance.P
        self.Dip = integrated_instance.Dip
        self.Dpi = integrated_instance.Dpi
        self.v = integrated_instance.v
        self.O = integrated_instance.O
        self.IO = integrated_instance.IO
        self.picking_time = integrated_instance.picking_time
        self.queue_length = integrated_instance.queue_length
        self.sumIO = integrated_instance.sumIO
    
    def build_gurobi_model(self, model):
        """ build gurobi model with obj and cons """
        M = self.bigM

        # 添加决策变量
        # 订单任务分拨上墙的决策变量
        tos_list = [o for o in range(self.O)]
        tos = model.addVars( tos_list, vtype=GRB.CONTINUOUS, name="tos")
        toe_list = [o for o in range(self.O)]
        toe = model.addVars( toe_list, vtype=GRB.CONTINUOUS, name="toe")
        a1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        a1 = model.addVars(a1_list, vtype=GRB.BINARY, name="a1")
        b1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        b1 = model.addVars(b1_list, vtype=GRB.BINARY, name="b1")
        c1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        c1 = model.addVars(c1_list, vtype=GRB.BINARY, name="c1")
        d1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        d1 = model.addVars(d1_list, vtype=GRB.BINARY, name="d1")
        # picking 决策变量
        x_list = [(i,j) for i in self.N for j in self.N]
        x = model.addVars(x_list, vtype=GRB.BINARY, name="x") # 车的路径
        Q_list = [i for i in self.N]
        Q = model.addVars( Q_list, vtype=GRB.CONTINUOUS, name="Q")  # 车的载重
        T_list = [i for i in self.N]
        T = model.addVars(T_list, vtype=GRB.CONTINUOUS, name="T")  # 车的时间
        pass_list = [(i,k) for i in self.N for k in self.K]
        passX = model.addVars( pass_list, vtype=GRB.BINARY, name="passX")  # 车k是否经过点i
        FT = model.addVar( vtype=GRB.CONTINUOUS, name="FT") # 所有任务完成的时间
        # sorting 决策变量
        I_list = [i for i in range(self.n)]
        I = model.addVars( I_list, vtype=GRB.INTEGER, name="I")  # 料箱i的初始到达输送机的时间
        y_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        y = model.addVars(y_list, vtype=GRB.BINARY, name="y") # 料箱i是否被分配给了拣选站p
        z_list = [(o,p) for o in range(self.O) for p in range(self.P)]
        z = model.addVars(z_list, vtype=GRB.BINARY, name="z") # 订单o是否被分配给了拣选站p
        Ta_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Ta = model.addVars(Ta_list, vtype=GRB.INTEGER, name="Ta") # 料箱i到达拣选站p的时间
        Ts_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Ts = model.addVars(Ts_list, vtype=GRB.INTEGER, name="Ts")  # 料箱i在拣选站p的开始拣选时间
        Te_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Te = model.addVars(Te_list, vtype=GRB.INTEGER, name="Te" ) # 料箱i在拣选站p的结束拣选时间
        f_list = [(i,j,p) for i in range(self.n) for j in range(self.n) for p in range(self.P)]
        f = model.addVars( f_list, vtype=GRB.BINARY, name="f") # 料箱i是否先于料箱j到达拣选站p


        # 添加目标函数
        model.modelSense = GRB.MINIMIZE
        model.setObjective( FT ) # 最小化最大完成时间目标
        


        # 添加约束条件
        # 0. 最大完成时间约束
        model.addConstrs( FT >= T[i] for i in self.N)
        # picking 约束条件
        # 1. 车辆的流平衡约束
        model.addConstrs( gp.quicksum( x[i,j] for j in self.N if j != i ) == gp.quicksum( x[j,i] for j in self.N if j != i ) for i in self.N)
        # 2. 车辆要完成所有任务
        model.addConstrs( gp.quicksum( x[i,j] for j in self.N if j != i) >= 1 for i in (self.P1 + self.P2 + self.D1 + self.D2))
        # 3. 同一个任务用同一个车
        model.addConstrs( passX[i,k] - passX[j,k] >= M * (x[i,j] - 1) for i in self.N for j in self.N for k in self.K)
        model.addConstrs( passX[i,k] - passX[j,k] <= M * (1 - x[i,j]) for i in self.N for j in self.N for k in self.K)
        # model.addConstrs( (x[i, j] == 1) >> (passX[i, k] == passX[j, k]) for i in self.N for j in self.N for k in self.K )
        model.addConstrs( passX[self.W[k], k] == 1 for k in self.K)
        model.addConstrs( gp.quicksum(passX[i, k] for k in self.K) == 1 for i in self.N)
        model.addConstrs( passX[i, k] == passX[i+2*self.n, k] for i in self.P1+self.P2 for k in self.K)
        # 4. 一个车只能从自己的出发点出发一次
        model.addConstrs( gp.quicksum( x[self.W[k],j] for j in self.N if j != self.W[k]) <= 1 for k in self.K)
        # 5. 载重约束
        model.addConstrs( Q[j] >= Q[i] + self.nodes[i]["demand"] - self.Q * (1 - x[i, j]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j)
        # model.addConstrs( (x[i, j] == 1) >> (Q[j] >= Q[i] + self.nodes[i]["demand"]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j)
        model.addConstrs( Q[i] >= 0 for i in self.N)
        model.addConstrs( Q[i] <= self.Q for i in self.N)
        # 6. 时间约束
        model.addConstrs( T[j] >= T[i] + self.timeMatrix[i][j] + self.nodes[i]["serviceTime"] - M * (1 - x[i, j]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j)
        # model.addConstrs( (x[i, j] == 1) >> (T[j] >= T[i] + self.timeMatrix[i][j] + self.nodes[i]["serviceTime"]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j)
        model.addConstrs( T[i] >= self.nodes[i]["readyTime"] for i in self.N )
        model.addConstrs( T[i] <= self.nodes[i]["dueTime"] for i in self.N )
        # 7. 到达终点的时间>=起点+服务+路程时间
        model.addConstrs( T[2 * self.n + i] >= T[i] + self.timeMatrix[i][i+2*self.n] + self.nodes[i]["serviceTime"] for i in (self.P1 + self.P2) )
        # sorting 约束条件
        # 8. 到达P2的时间>=到达环形输送机出口的时间
        model.addConstrs( T[i - self.n] >= Te[i - 2 * self.n, self.P-1] + (self.Dpi[i - 2 * self.n][self.P-1]/self.v) for i in self.D1 )
        # model.addConstrs( T[i] == 3000 for i in self.P2)
        # 9. 到达输送机的时间要>=到达D1的时间
        model.addConstrs( I[i] == T[i + 2 * self.n] for i in range(self.n) )
        # 10. 控制决策变量f的两条约束
        model.addConstrs( Ta[i,p] - Ta[j,p] <= (1 - f[i,j,p]) * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        model.addConstrs( Ta[i,p] - Ta[j,p] >= -f[i,j,p] * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        # 11. 先到达拣选站p的料箱i的结束拣选时间要小于等于后到达拣选站p的料箱j的开始拣选时间
        model.addConstrs( Ts[j,p] - Te[i,p] >= M * (f[i,j,p] + y[i,p] + y[j,p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        model.addConstrs( Ts[i,p] - Te[j,p] >= M * ((1- f[i,j,p]) + y[i,p] + y[j,p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        # 12. 只有订单o被分配给了拣选站p & 料箱i属于订单o，那么料箱i一定能被分配给拣选站p
        model.addConstrs( self.IO[i][o] * z[o,p] <= y[i,p] for i in range(self.n) for p in range(self.P) for o in range(self.O))
        # 13. 所有任务都要被完成
        model.addConstrs( gp.quicksum( z[o,p] for p in range(self.P)) == 1 for o in range(self.O))
        model.addConstr( gp.quicksum( y[i,p] for i in range(self.n) for p in range(self.P)) <= self.sumIO)
        # 14. 料箱i的结束拣选时间一定大于等于它的开始拣选时间（当yip=0时）
        model.addConstrs( Te[i,p] == Ts[i,p] + self.picking_time * y[i,p] for i in range(self.n) for p in range(self.P))
        # 15. 当料箱i不去拣选站p时，Ts=Ta
        model.addConstrs( Ts[i,p] <= Ta[i,p] + M * y[i,p] for i in range(self.n) for p in range(self.P))
        # 16. 料箱i到达第一个拣选站p=0时的时间（初始化）：
        model.addConstrs( Ta[i,0] == I[i] + self.Dip[i][0]/self.v for i in range(self.n))
        # 17. 料箱到达下一个拣选站的时间为：在上一个拣选站结束拣选的时间+路程时间（----------标记）约束条件3和约束条件2二选一就可以，到底是大于等于还是等于？
        model.addConstrs( Ta[i,p] == Te[i,p-1] + (self.Dip[i][p] - self.Dip[i][p-1])/self.v for i in range(self.n) for p in range(1,self.P))
        # 18. 开始拣选时间的约束——料箱i在p开始拣选的时间一定大于or等于在上一个拣选站结束拣选的时间+路程时间
        model.addConstrs( Ts[i, p] >= Ta[i, p] for i in range(self.n) for p in range(self.P))
        # 19. 拣选站处的缓存区大小约束
        model.addConstrs( Ts[i,p] - Ta[i,p] <= (self.queue_length-1) * self.picking_time for i in range(self.n) for p in range(self.P))
        # 20. 添加订单任务分拨上墙的约束条件
        model.addConstrs( tos[o1] - tos[o2] <= a1[o1,o2] * self.bigM for o1 in range(self.O) for o2 in range(self.O))
        model.addConstrs( toe[o2] - tos[o1] <= b1[o1,o2] * self.bigM for o1 in range(self.O) for o2 in range(self.O))
        model.addConstrs( tos[o] <= toe[o] for o in range(self.O))
        model.addConstrs( c1[o1,o2] >= a1[o1,o2] + b1[o1,o2] - 1 for o1 in range(self.O) for o2 in range(self.O))
        model.addConstrs( d1[o1,o2] >= c1[o1,o2] + z[o1,p] +z[o2,p] - 2 for o1 in range(self.O) for o2 in range(self.O) for p in range(self.P))
        model.addConstrs( gp.quicksum( d1[o1,o2] for o2 in range(self.O)) <= 8 for o1 in range(self.O))
        # 添加订单任务分拨上墙与任务分配和机器人调度的衔接约束条件
        model.addConstrs( toe[o] - Te[i,p] >= (self.IO[i][o] + y[i,p] + z[o,p] - 3) * self.bigM for o in range(self.O) for i in range(self.n) for p in range(self.P))
        model.addConstrs( tos[o] - Ts[i,p] <= (3 - self.IO[i][o] - y[i,p] - z[o,p]) * self.bigM for o in range(self.O) for i in range(self.n) for p in range(self.P))
        model.update()

        info = {}
        info["x"] = x
        info["z"] = z

        return info

    def set_init_solution(self, model):
        """ set init solution for model """
        x_val = np.zeros((self.integrated_instance.nodeNum, self.integrated_instance.nodeNum))
        # init strategy 1
        init_alg = NNH_heuristic_algorithm(self.integrated_instance)
        routes = init_alg.NNH_main()
        # init strategy 2
        # set init solution
        for route in routes:
            for i in range(1, len(route)):
                pi = route[i-1]
                pj = route[i]
                x_val[pi, pj] = 1
            x_val[route[-1], route[0]] = 1
        for i in range(self.integrated_instance.nodeNum):
            for j in range(self.integrated_instance.nodeNum):
                model.getVarByName("x[{},{}]".format(i,j)).start = x_val[i,j]
        model.update()

    def run_gurobi_model(self):
        model = gp.Model("IntegratedGurobiModel") # 创建gurobi模型
        self.build_gurobi_model(model) # 构建gurobi模型
        if self.time_limit is not None: # 求解时间限制
            model.setParam("TimeLimit", self.time_limit)
        model.setParam("OutputFlag", 1)  # 求解过程展示
        if self.init_flag: # 设置gurobi模型初始解
            self.set_init_solution(model)
        model.optimize() # 求解模型

        return model

if __name__ == "__main__":
    w_num = 6
    l_num = 4
    bins_num = 9
    robot_num = 9
    picking_station_num = 6
    orders_num = 5
    problem = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    solver = IntegratedGurobiModel(problem, init_flag=False)
    model = solver.run_gurobi_model()





