from linkage_physics import *
from ga import GA
import pygad, numpy, copy, pickle

# Global Parameters
MAX_NODE = 6

class LinkageGA(GA):
    def __init__(self, k=MAX_NODE, b=10, max_trial=100,
                 num_generations=1000, num_parents_mating=5, sol_per_pop=10,
                 parent_selection_type="sss", keep_parents=2,
                 crossover_type="single_point", mutation_type="random",
                 mutation_percent_genes=10):
        
        super().__init__(self.fitness_func, self.generate_solution, k, b, max_trial, num_generations, 
                         num_parents_mating, sol_per_pop, parent_selection_type, keep_parents, 
                         crossover_type, mutation_type, mutation_percent_genes) 

    def data_to_solution(self, data):
        solution = data[:]
        cur_node_num = self.get_node_num(data)
        if cur_node_num < MAX_NODE:
            solution += [-1, -1, -1, -1, -1] * (MAX_NODE - cur_node_num)
        return solution

    def solution_to_data(self, solution):
        node_num = len(solution) // 5
        while node_num > 0 and solution[node_num * 5 - 1] == GA.NOT_USED:
            node_num -= 1
        data = solution[:node_num * 5]
        data[0] = -1
        data[4] = int(data[4])
        for i in range(1, node_num):
            data[5 * i + 0] = int(data[5 * i + 0])
            data[5 * i + 1] = int(data[5 * i + 1])
            data[5 * i + 4] = int(data[5 * i + 4])
        return data

    def get_node_num(self, data):
        return len(data) // 5

    def set_to_linkage(self, data):
        link = Linkage(self.get_node_num(data))
        link.rad_motor = data[1]
        link.ctr_motor = (data[2], data[3])
        for i in range(1, self.get_node_num(data)):
            link.U[i + 1] = 1
            state = data[i * 5 + 4]
            if state == GA.MOVABLE:
                link.F[i + 1] = 0
                link.C1[i + 1] = int(data[i * 5 + 0] + 1) 
                link.C2[i + 1] = int(data[i * 5 + 1] + 1)
                link.len1[i + 1] = data[i * 5 + 2]
                link.len2[i + 1] = data[i * 5 + 3]
            elif state == GA.FIXED:
                link.F[i + 1] = 1
                link.node_position[i + 1] = (data[i * 5 + 2], data[i * 5 + 3])
            else:
                assert False
        return link

    def check_topology_feasibility(self, data):
        node_num = len(data) // 5
        visited = [False for i in range(node_num)]
        visited[node_num - 1] = True
        stack = [node_num - 1]
        iter = 0
        while len(stack) > 0 and iter < 100:
            id = int(stack[-1]) # Cast to integer
            stack.pop()
            if data[id * 5 + 4] == GA.FIXED or id == 0: 
                ss = []
            else:
                ss = [data[id * 5 + 0], data[id * 5 + 1]]
            for i in ss:
                stack.append(int(i))
                visited[int(i)] = True
            iter += 1
            if (iter == 100): print("warning, inf.loop:", data)
        if any(not v for v in visited):
            return False

        visited = [False for i in range(node_num)]
        visited[0] = True
        stack = [0]
        iter = 0
        while len(stack) > 0 and iter < 100:
            id = stack[-1]
            stack.pop()
            for j in range(id + 1, node_num):
                if data[j * 5 + 0] == id or data[j * 5 + 1] == id:
                    stack.append(j)
                    visited[j] = True
            iter += 1
            if (iter == 100): print("warning, inf.loop:", data)
        if any(data[i * 5 + 4] == GA.MOVABLE and not visited[i] for i in range(node_num)):
            return False
        return True

    def check_geometry_feasibility(self, data, nrSample=32):
        link = self.set_to_linkage(data)
        for i in range(nrSample):
            theta, succ = link.forward_kinematic(math.pi * 2 * i / nrSample)
            if not succ:
                return False
            if any(pos[d] < -self.B or pos[d] > self.B for pos in link.node_position for d in range(2)):
                return False
        return True

    def check_data_validity(self, data, debug=False):
        for i in range(len(data) // 5):
            state = data[i * 5 + 4]
            first_connect = data[i * 5]
            second_connect = data[i * 5 + 1]
            if i == 0:
                if state != GA.MOVABLE:
                    if debug: print("motor not movable")
                    return False
                if first_connect != -1:
                    if debug: print("motor first connect wrong")
                    return False
            elif i == 1:
                if state != GA.FIXED:
                    if debug: print("first node not fixed")
                    return False
                if first_connect != -1:
                    if debug: print("first node first connect wrong")
                    return False
                if second_connect != -1:
                    if debug: print("first node second connect wrong")
                    return False
            else:
                if state == GA.FIXED:
                    if first_connect != -1:
                        if debug: print("FIXED node first connect wrong")
                        return False
                    if second_connect != -1:
                        if debug: print("FIXED node second connect wrong")
                        return False
                elif state == GA.MOVABLE:
                    if first_connect == second_connect:
                        if debug: print("connects to same node")
                        return False
                    if first_connect == -1:
                        if debug: print("movable node first connect wrong")
                        return False
                    if second_connect == -1:
                        if debug: print("movable node second connect wrong")
                        return False
                else:
                    if debug: print("there is a not used point in structure")
                    return False
        return True

    def check_feasibility(self, data, min_node_num=0):
        return self.check_data_validity(data) \
               and self.check_topology_feasibility(data) \
               and self.check_geometry_feasibility(data) \
               and len(data) // 5 > min_node_num   

    def generate_solution(self):
        while True:
            data = self.generate_init_state()
            if self.check_feasibility(data, min_node_num=3):
                break
        return self.data_to_solution(data)

    def fitness_func(self, ga_instance, solution, solution_idx):
        data = self.solution_to_data(solution)
        if not self.check_feasibility(data):
            return -1
        link = self.set_to_linkage(data)
        robot = create_robot(link, sep=5.)
        if robot is None:
            return 0.
        else:
            return robot.eval_performance(10.)     

if __name__ == '__main__':
    linkage_ga = LinkageGA(k=MAX_NODE, b=10, max_trial=1000,
                 num_generations=5000, num_parents_mating=40, sol_per_pop=100,
                 parent_selection_type="sss", keep_parents=20,
                 crossover_type="single_point", mutation_type="random",
                 mutation_percent_genes=20)
    
    linkage_ga.run()
    solution, solution_fitness, solution_idx = linkage_ga.get_best_solution()

    with open('best_ga.pickle', 'wb') as handle:
        pickle.dump(solution, handle)

    data = linkage_ga.solution_to_data(solution)
    num_nodes = linkage_ga.get_node_num(data)

    print("\nBest Solution Parameters:")
    print("-" * 50)
    print(f"{'Node':<8}{'C1':<8}{'C2':<8}{'Len1':<8}{'Len2':<8}{'State':<8}")
    print("-" * 50)
    print(f"{'Motor':<8}{data[1]:<8.2f}{'':<8}{'':<8}{'':<8}{'':<8}")  
    for i in range(1, num_nodes):
        print(f"{i+1:<8}{data[i * 5 + 0] + 1:<8.0f}{data[i * 5 + 1] + 1:<8.0f}{data[i * 5 + 2]:<8.2f}{data[i * 5 + 3]:<8.2f}{'Fixed' if data[i * 5 + 4] == GA.FIXED else 'Movable':<8}")
    print("-" * 50)
    print(f"Fitness: {solution_fitness:.2f}")