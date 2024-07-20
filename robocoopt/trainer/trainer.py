import os, pickle, time, json
from PyQt5.QtCore import QThread, pyqtSignal
from robocoopt.linkage.linkage_physics import create_robot
from robocoopt.opt.anneal.optimizer_anneal import LinkageAnnealer

PICKLE_FOLDER = 'out/pickle/temp'
HIGHSCORE_FILE = 'out/pickle/best.pickle'
ALL_SCORES_FILE = os.path.join(PICKLE_FOLDER, 'scores.json')

if not os.path.exists(PICKLE_FOLDER):
    os.makedirs(PICKLE_FOLDER)

def run_annealing_once():
    opt = LinkageAnnealer()
    opt.steps = 100
    state, e = opt.anneal()

    pickle_filename = os.path.join(PICKLE_FOLDER, f'{time.time()}.pickle')
    with open(pickle_filename, 'wb') as f:
        pickle.dump(state, f)

    link = opt.set_to_linkage()
    robo = create_robot(link, sep=5.)

    if robo is None:
        return -1, None, None
    
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

    def __init__(self):
        super().__init__()
        self.is_running = False
        self.scores = load_scores_json()
        self.high_score = max(self.scores.values(), default=0)

    def start_training(self):
        self.is_running = True
        self.start()

    def stop_training(self):
        self.is_running = False

    def run(self):
        while self.is_running:
            score, pickle_filename, state = run_annealing_once()
            if score == -1:
                continue  

            self.scores[pickle_filename] = score
            self.update_scores.emit(self.high_score, list(self.scores.values()))
            save_score_json(self.scores)

            if score > self.high_score:
                self.high_score = score
                with open(HIGHSCORE_FILE, 'wb') as handle:
                    pickle.dump(state, handle)