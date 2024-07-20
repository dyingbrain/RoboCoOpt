import os, pickle, time, json
from PyQt5.QtCore import QThread, pyqtSignal
from robocoopt.linkage.linkage_physics import create_robot
from robocoopt.opt.anneal.optimizer_anneal import LinkageAnnealer
from robocoopt.opt.ga.optimizer_ga import LinkageGA, data_to_solution, solution_to_data

PICKLE_FOLDER = 'out/pickle/temp'
ALL_SCORES_FILE = os.path.join(PICKLE_FOLDER, 'scores.json')

if not os.path.exists(PICKLE_FOLDER):
    os.makedirs(PICKLE_FOLDER)

def run_optimization_once(algorithm='SA'):
    score = -1
    pickle_filename = None
    state = None

    if algorithm == 'SA':
        opt = LinkageAnnealer()
        opt.steps = 100
        state, e = opt.anneal()
        link = opt.set_to_linkage()

    elif algorithm == 'GA':
        ga = LinkageGA()  
        ga.run()
        solution, solution_fitness, _ = ga.get_best_solution()
        state = solution_to_data(solution).state
        link = solution_to_data(solution).set_to_linkage()

    else:
        print("Invalid algorithm selected.")
        return score, pickle_filename, state

    pickle_filename = os.path.join(PICKLE_FOLDER, f'{time.time()}.pickle')
    with open(pickle_filename, 'wb') as f:
        pickle.dump(state, f)
    
    robo = create_robot(link, sep=5.)

    if robo is not None:
        score = robo.eval_performance(10.)
        
    return score, pickle_filename, state

def save_score_json(scores):
    with open(ALL_SCORES_FILE, 'w') as f:
        json.dump(scores, f)

def load_scores_json():
    if os.path.exists(ALL_SCORES_FILE):
        with open(ALL_SCORES_FILE, 'r') as f:
            return json.load(f)
    return {}

class TrainingThread(QThread):
    update_scores = pyqtSignal(float, list)

    def __init__(self, algorithm='SA'): 
        super().__init__()
        self.is_running = False
        self.algorithm = algorithm 
        self.scores = load_scores_json()
        self.high_score = max(self.scores.values(), default=0)

    def start_training(self, algorithm):  
        self.algorithm = algorithm  
        self.is_running = True
        self.start()

    def stop_training(self):
        self.is_running = False

    def run(self):
        while self.is_running:
            score, pickle_filename, state = run_optimization_once(self.algorithm) 
            if score == -1:
                continue

            self.scores[pickle_filename] = score
            self.update_scores.emit(self.high_score, list(self.scores.values()))
            save_score_json(self.scores)

            if score > self.high_score:
                self.high_score = score
                if self.algorithm == 'SA':
                    filename = 'out/pickle/best_sa.pickle'
                else:  
                    filename = 'out/pickle/best_ga.pickle' 
                with open(filename, 'wb') as handle:
                    pickle.dump(state, handle)