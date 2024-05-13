'''
File: rule_based_algorithm_for_order.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a rule based algorithm for the init solution of the order.
----------
Author: 626
Created Date: 2024.03.19
hhh
'''


import sys
sys.path.append('..')
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance


class RuleBasedAlgorithmForOrder():
    def __init__(self, instance: Instance) -> None:
        self.O = instance.O
        self.P = instance.P
    
    def main(self):
        """ input instance, use rule based algorithm to generate the init solution for the order. """
        assignment_solution = [0 for _ in range(self.O)]
        # add order to the picking station.
        p = 0
        for o in range(self.O):
            assignment_solution[o] = p
            p += 1
            if p == self.P:
                p = 0
        
        return assignment_solution

    def runner(self):
        """ runner of the rule based algorithm for order. """
        assignment_solution = self.main()

        return assignment_solution
    

if __name__ == "__main__":
    w_num = 8
    l_num = 8
    bins_num = 10
    robot_num = 3
    picking_station_num = 5
    orders_num = 3
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    algorithm_tools = RuleBasedAlgorithmForOrder(instance)
    assignment_solution = algorithm_tools.runner()
    print(assignment_solution)