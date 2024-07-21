from robocoopt.linkage.linkage_physics import *
from robocoopt.util.utility import *
import pygad, numpy, pickle, copy, random, math

class LinkageGA():
    MAX_NODE = 6
    NOT_USED = -1
    MOVABLE = 0
    FIXED = 1
    K = MAX_NODE
    B = 10
    maxTrial = 100
    maxDist = 2.0 * math.sqrt(2.0) * B

    def __init__(self, state=[], K=5, B=10, num_generations=50, num_parents_mating=10, sol_per_pop=200,
                 parent_selection_type="sss", keep_parents=2, num_threads=32):
        
        self.state = state
        self.K = K
        self.B = B
        self.maxTrial = 100

        if len(self.state) == 0:
            while True:
                self.state = self.generate_init_state()
                if self.check_topology_feasibility() and self.check_geometry_feasibility():
                    break

        self.num_generations = num_generations
        self.num_parents_mating = num_parents_mating
        self.sol_per_pop = sol_per_pop
        self.parent_selection_type = parent_selection_type
        self.keep_parents = keep_parents
        self.gene_space = self.generate_gen_space()
        self.num_threads = num_threads

    def generate_init_state(self, ret=None):
        if ret is None:
            ret = []
        maxDist = 2.0 * math.sqrt(2.0) * self.B
        for i in range(len(ret) // 5, self.K):
            if i == 0:
                state = self.MOVABLE
            elif i == 1:
                state = random.choice([self.NOT_USED, self.FIXED])
            else:
                state = random.choice([self.NOT_USED, self.MOVABLE, self.FIXED])
            if state == self.NOT_USED:
                break
            elif state == self.FIXED:
                ret += [-1, -1, random.uniform(-self.B, self.B), random.uniform(
                    -self.B, self.B), state]
            elif i == 0:
                ret += [-1, 1, 0.0, 0.0, state]
            else:
                s = [-1, -1, -1, -1, -1]
                while s[0] == s[1]:
                    s = [random.randint(0, i - 1), random.randint(0, i - 1),
                         random.uniform(0, maxDist), random.uniform(0, maxDist), state]
                ret += s
        while ret[-1] == self.FIXED:
            ret = ret[0:len(ret) - 5]
        return ret

    def check_feasibility(self, min_node_num=0):
        return self.check_data_validity() \
            and self.check_topology_feasibility() \
            and self.check_geometry_feasibility() \
            and self.nrN() > min_node_num

    def check_data_validity(self, debug=False):
        debug = True
        data = self.state
        for i in range(self.nrN()):
            state = data[i * 5 + 4]
            first_connect = data[i * 5]
            second_connect = data[i * 5 + 1]
            if i == 0:
                if state is not LinkageGA.MOVABLE:
                    if debug:
                        print("motor not movable")
                    return False
                if first_connect != -1:
                    if debug:
                        print("motor first connect wrong")
                    return False
            elif i == 1:
                if state is not LinkageGA.FIXED:
                    if debug:
                        print("first node not fixed")
                    return False
                if first_connect != -1:
                    if debug:
                        print("first node first connect wrong")
                    return False
                if second_connect != -1:
                    if debug:
                        print("first node second connect wrong")
                    return False
            else:
                if state is LinkageGA.FIXED:
                    if first_connect != -1:
                        if debug:
                            print("FIXED node first connect wrong")
                        return False
                    if second_connect != -1:
                        if debug:
                            print("FIXED node second connect wrong")
                        return False
                elif state is LinkageGA.MOVABLE:
                    if first_connect == second_connect:
                        if debug:
                            print("connectes to same node")
                        return False
                    if first_connect == -1:
                        if debug:
                            print("movable node first connect wrong")
                        return False
                    if second_connect == -1:
                        if debug:
                            print("movable node second connect wrong")
                        return False
                else:
                    if debug:
                        print("there is a not used point in structure")
                    return False
        return True

    def check_topology_feasibility(self):
        if self.state[-1] != self.MOVABLE or len(self.state) // 5 < 3:
            return False
        visited = [False for i in range(self.nrN())]
        visited[self.nrN() - 1] = True
        stack = [self.nrN() - 1]
        while len(stack) > 0:
            id = stack[-1]
            stack.pop()
            if self.state[id * 5 + 4] == self.FIXED or id == 0:
                ss = []
            else:
                ss = [self.state[id * 5 + 0], self.state[id * 5 + 1]]
            for i in ss:
                stack.append(i)
                visited[i] = True
        for i in range(self.nrN()):
            if not visited[i]:
                return False
        visited = [False for i in range(self.nrN())]
        visited[0] = True
        stack = [0]
        while len(stack) > 0:
            id = stack[-1]
            stack.pop()
            for j in range(id + 1, self.nrN()):
                if self.state[j * 5 + 0] == id or self.state[j * 5 + 1] == id:
                    stack.append(j)
                    visited[j] = True
        for i in range(self.nrN()):
            if self.state[i * 5 + 4] == self.MOVABLE and not visited[i]:
                return False
        return True

    def check_geometry_feasibility(self):
        self.link = self.set_to_linkage()
        return self.link.check_geometry_feasibility()

    def change_geometry(self):
        ids = []
        for i in range(1, self.nrN()):
            ids += [i * 5 + 2, i * 5 + 3]
        id = random.choice(ids)
        ctr = self.state[id]
        delta = self.B
        trial = 0
        while trial < self.maxTrial:
            val = random.uniform(ctr - delta, ctr + delta)
            self.state[id] = val
            if self.check_geometry_feasibility():
                break
            trial = trial + 1
        return trial < self.maxTrial

    def add_node(self):
        if len(self.state) == self.K * 5:
            return False
        nrN0 = self.nrN()
        state0 = copy.deepcopy(self.state)
        trial = 0
        while trial < self.maxTrial:
            self.state = self.generate_init_state(ret=copy.deepcopy(state0))
            if self.check_topology_feasibility() and self.check_geometry_feasibility() and self.nrN() > nrN0:
                break
            trial = trial + 1
        return trial < self.maxTrial

    def remove_node(self):
        if len(self.state) == 5:
            return False
        self.state = self.state[0:len(self.state) - 5]
        return self.check_topology_feasibility() and self.check_geometry_feasibility()

    def set_to_linkage(self):
        link = Linkage(self.nrN())
        link.rad_motor = self.state[1]
        link.ctr_motor = (self.state[2], self.state[3])
        for i in range(1, self.nrN()):
            link.U[i + 1] = 1
            state = self.state[i * 5 + 4]
            if state == self.MOVABLE:
                link.F[i + 1] = 0
                link.C1[i + 1] = self.state[i * 5 + 0] + 1
                link.C2[i + 1] = self.state[i * 5 + 1] + 1
                link.len1[i + 1] = self.state[i * 5 + 2]
                link.len2[i + 1] = self.state[i * 5 + 3]
            elif state == self.FIXED:
                link.F[i + 1] = 1
                link.node_position[i + 1] = (
                    self.state[i * 5 + 2], self.state[i * 5 + 3])
            else:
                assert False
        return link

    def nrN(self):
        return len(self.state) // 5

    def move(self):
        self.mutation()

    def mutation(self):
        state0 = copy.deepcopy(self.state)
        while True:
            self.state = copy.deepcopy(state0)
            op_type = random.randint(0, 2)
            if op_type == 0:
                if self.change_geometry():
                    break
            elif op_type == 1:
                if self.add_node():
                    break
            else:
                assert op_type == 2
                if self.remove_node():
                    break

    def cross_over_once(self, mother, father):
        nrN_mother = len(mother) // 5
        nrN_father = len(father) // 5
        nrN_max = max(nrN_mother, nrN_father)
        result = []
        for i in range(nrN_max):
            choice = random.randint(0, 1)
            if choice == 0:
                if len(mother) < i * 5 + 5:
                    break
                result += mother[i * 5:i * 5 + 5]
            else:
                if len(father) < i * 5 + 5:
                    break
                result += father[i * 5:i * 5 + 5]
        return result

    def cross_over(self, mother, father):
        while True:
            self.state = self.cross_over_once(mother, father)
            if self.check_geometry_feasibility() and self.check_geometry_feasibility():
                break

    def energy(self):
        link = self.set_to_linkage()
        robot = create_robot(link, sep=5.)
        if robot is None:
            return 0.
        else:
            return -robot.eval_performance(10.)
    @staticmethod
    def data_to_solution(linkage):
        solution = linkage.state
        cur_node_num = linkage.nrN()
        if cur_node_num < LinkageGA.MAX_NODE:
            for i in range(LinkageGA.MAX_NODE - cur_node_num):
                solution += [-1, -1, -1, -1, -1]
        return np.array(solution)
    
    @staticmethod
    def solution_to_data(solution):
        if type(solution) is not list:
            solution = solution.tolist()
        node_num = len(solution) // 5
        while (int(solution[node_num * 5 - 1]) is LinkageGA.NOT_USED) and node_num > 0:
            node_num = node_num - 1
        data = solution[:node_num * 5]
        data[0] = -1
        data[4] = int(data[4])
        for i in range(1, node_num):
            data[5 * i + 0] = int(data[5 * i + 0])
            data[5 * i + 1] = int(data[5 * i + 1])
            data[5 * i + 4] = int(data[5 * i + 4])
        linkage = LinkageGA(state=copy.deepcopy(data))
        return linkage
    
    @staticmethod
    def generate_solution():
        linkage = LinkageGA()
        link = linkage.set_to_linkage()
        robot = create_robot(link, sep=5.)  
        return LinkageGA.data_to_solution(linkage)
    
    @staticmethod
    def generate_population(sol_per_pop):
        print("Start generating initial population")
        print("-" * 70)
        print(f"{'Index':<8}{'Solution':<45}{'Fitness':>12}")
        print("-" * 70)

        initial_population = []
        for i in range(sol_per_pop):
            solution = LinkageGA.generate_solution()
            initial_population.append(solution)
            fitness = LinkageGA.fitness_func(None, solution, i)
            print(
                f"{i:<8}{str(solution)[:40] + '...' if len(str(solution)) > 40 else str(solution):<45}{fitness:>12.5f}"
            )

        print("-" * 70)
        print("Initial population generated!")

        return initial_population
    
    @staticmethod
    def generate_gen_space():
        gene_space = [[-1], random.uniform(
            LinkageGA.B / 10, LinkageGA.B), [0.0], [0.0], [0]]
        gene_space += [[-1], [-1], random.uniform(
            -LinkageGA.B, LinkageGA.B), random.uniform(-LinkageGA.B, LinkageGA.B), [1]]
        for i in range(2, LinkageGA.K):
            gene_space += [random.randint(-1, i - 1), random.randint(-1, i - 1),
                          random.uniform(0, LinkageGA.maxDist), random.uniform(0, LinkageGA.maxDist), [-1, 0, 1]]
        return gene_space
    
    @staticmethod
    def fitness_func(ga_instance, solution, solution_idx):
        linkage = LinkageGA.solution_to_data(solution)
        if not linkage.check_feasibility():
            return -1
        link = linkage.set_to_linkage()
        robot = create_robot(link, sep=5.)
        if robot is None:
            return 0.
        else:
            fitness = robot.eval_performance(10.)
            print("Fitness:", robot.eval_performance(10.))
            return fitness
        
    @staticmethod
    def crossover_func(parents, offspring_size, ga_instance):
        offspring = []
        idx = 0
        while len(offspring) != offspring_size[0]:
            parent1 = parents[idx % parents.shape[0], :].copy()
            parent2 = parents[(idx + 1) % parents.shape[0], :].copy()

            linkageP1 = LinkageGA.solution_to_data(parent1)
            linkageP2 = LinkageGA.solution_to_data(parent2)
            linkageChild = LinkageGA()

            linkageChild.cross_over(linkageP1.state, linkageP2.state)
            child = LinkageGA.data_to_solution(linkageChild)

            offspring.append(child)
            idx += 1

        return numpy.array(offspring)
    
    @staticmethod
    def mutation_func(offspring, ga_instance):
        for chromosome_idx in range(offspring.shape[0]):
            linkage = LinkageGA.solution_to_data(offspring[chromosome_idx])
            linkage.mutation()
            offspring[chromosome_idx] = LinkageGA.data_to_solution(linkage)
        return offspring

    def run(self):
        initial_population = self.generate_population(self.sol_per_pop)
        ga_instance = pygad.GA(
            num_generations=self.num_generations,
            num_parents_mating=self.num_parents_mating,
            fitness_func=self.fitness_func,
            initial_population=initial_population,
            parent_selection_type=self.parent_selection_type,
            keep_parents=self.keep_parents,
            crossover_type=self.crossover_func,
            mutation_type=self.mutation_func,
            parallel_processing=["process", self.num_threads],
            gene_space=self.gene_space
        )

        ga_instance.run()
        return ga_instance
    
    @staticmethod
    def visualize_results(ga_instance):
        solution, solution_fitness, solution_idx = ga_instance.best_solution()
        print("Parameters of the best solution : {solution}".format(
            solution=solution))
        print("Fitness value of the best solution = {solution_fitness}".format(
            solution_fitness=solution_fitness))

        linkage = LinkageGA.solution_to_data(solution)

        with open(pickle_file_path('best_ga.pickle'), 'wb') as handle:
            pickle.dump(linkage.state, handle)

        link = linkage.set_to_linkage()
        robot = create_robot(link, sep=5.)
        main_linkage_physics(robot)

def main():
    linkage_ga = LinkageGA()
    linkage_inst = linkage_ga.run()
    linkage_ga.visualize_results(linkage_inst)

if __name__ == '__main__':
    main()