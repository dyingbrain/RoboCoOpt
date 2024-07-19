import math
import random
import pygad

class GA:
    NOT_USED = -1
    MOVABLE = 0
    FIXED = 1

    def __init__(self, fitness_func, soln_generator, 
                 k=6, b=10, max_trial=1000,
                 num_generations=1000, num_parents_mating=4, sol_per_pop=10,
                 parent_selection_type="sss", keep_parents=1,
                 crossover_type="single_point", mutation_type="random",
                 mutation_percent_genes=20):

        self.fitness_func = fitness_func
        self.solution_generator = soln_generator

        self.K = k
        self.B = b

        self.maxTrial = max_trial
        self.maxDist = 2.0 * math.sqrt(2.0) * self.B

        self.num_generations = num_generations
        self.num_parents_mating = num_parents_mating
        self.sol_per_pop = sol_per_pop
        self.parent_selection_type = parent_selection_type
        self.keep_parents = keep_parents
        self.crossover_type = crossover_type
        self.mutation_type = mutation_type
        self.mutation_percent_genes = mutation_percent_genes

        self.initial_population = self.generate_population(
            sol_per_pop, 
            self.solution_generator, 
            self.fitness_func
        )
        
        self.gene_space = self.generate_gen_space()

    def generate_init_state(self, ret=None):
        if ret is None:
            ret = []
        for i in range(len(ret) // 5, self.K):
            if i == 0:
                state = GA.MOVABLE
            elif i == 1:
                state = random.choice([GA.NOT_USED, GA.FIXED])
            else:
                state = random.choice([GA.NOT_USED, GA.MOVABLE, GA.FIXED])

            if state == GA.NOT_USED:
                break
            elif state == GA.FIXED:
                ret += [-1.0, -1.0, random.uniform(-self.B, self.B), random.uniform(-self.B, self.B), float(state)] 
            elif i == 0:
                ret += [-1.0, random.uniform(self.B / 10, self.B), 0.0, 0.0, float(state)]
            else:
                s = [-1.0, -1.0, -1.0, -1.0, -1.0] 
                while s[0] == s[1]:
                    s = [random.randint(0, i - 1), random.randint(0, i - 1), random.uniform(0, self.maxDist),
                         random.uniform(0, self.maxDist), float(state)]
                ret += s

        while ret and ret[-1] == GA.FIXED:
            ret = ret[:-5]

        return ret

    def generate_population(self, sol_per_pop, solution_generator, fitness_function):
        print("Start generating initial population")
        print("-" * 70)  
        print(f"{'Index':<8}{'Solution':<45}{'Fitness':>12}")
        print("-" * 70)

        initial_population = []
        for i in range(sol_per_pop):
            solution = solution_generator() 
            initial_population.append(solution)
            fitness = fitness_function(None, solution, i)
            print(f"{i:<8}{str(solution)[:40] + '...' if len(str(solution)) > 40 else str(solution):<45}{fitness:>12.5f}")

        print("-" * 70) 
        print("Initial population generated")

        return initial_population

    def generate_gen_space(self):
        gene_space = [
            [-1], [self.B / 10, self.B], [0.0], [0.0], [0]  # Motor
        ]

        gene_space += [
            [-1], [-1], [-self.B, self.B], [-self.B, self.B], [1]  # First node
        ]

        for i in range(2, self.K):
            gene_space += [
                [j for j in range(-1, i)], [j for j in range(-1, i)],
                [0, self.maxDist], [0, self.maxDist], [-1, 0, 1]  # Other nodes
            ]

        return gene_space

    def run(self):
        self.ga_instance = pygad.GA(
            num_generations=self.num_generations,
            num_parents_mating=self.num_parents_mating,
            fitness_func=self.fitness_func,
            initial_population=self.initial_population,
            parent_selection_type=self.parent_selection_type,
            keep_parents=self.keep_parents,
            crossover_type=self.crossover_type,
            mutation_type=self.mutation_type,
            mutation_percent_genes=self.mutation_percent_genes,
            gene_space=self.gene_space
        )
        
        self.ga_instance.run()

    def get_best_solution(self):
        return self.ga_instance.best_solution()