'''
File: rule_based_algorithm_for_robot.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a rule based algorithm for the init solution of the robot.
----------
Author: 626
Created Date: 2024.03.19
'''


import sys
sys.path.append('..')
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance


class RuleBasedAlgorithmForRobot():
    def __init__(self, instance):
        self.n = instance.n
        self.R = instance.robotNum

    def main(self):
        """ input instance, use rule based algorithm to generate the init solution for the robot. """
        routes = [[] for _ in range(self.R)]
        # add robot's start point.
        cur_robot = 0
        for i in range(self.R):
            routes[i].append(self.n*4+i)
            cur_robot += 1
        # add p1 and d1 into route.
        cur_robot = 0
        for i in range(self.n):
            routes[cur_robot].append(i)
            routes[cur_robot].append(self.n*2+i)
            cur_robot += 1
            if cur_robot == self.R:
                cur_robot = 0
        # add p2 and d2 into route.
        cur_robot = 0
        for i in range(self.n):
            routes[cur_robot].append(i+self.n)
            routes[cur_robot].append(self.n*2+i+self.n)
            cur_robot += 1
            if cur_robot == self.R:
                cur_robot = 0
        
        return routes

    def runner(self):
        """ runner of the rule based algorithm for robot. """
        routes = self.main()

        return routes
        

if __name__ == "__main__":
    w_num = 8
    l_num = 8
    bins_num = 10
    robot_num = 3
    picking_station_num = 5
    orders_num = 3
    instance = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    algorithm_tools = RuleBasedAlgorithmForRobot(instance)
    routes = algorithm_tools.runner()
    print(routes)