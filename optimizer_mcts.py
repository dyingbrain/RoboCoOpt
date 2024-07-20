from mcts.base.base import BaseAction,BaseState
from optimizer_anneal import LinkageAnnealer
from parallel_mcts import ParallelMCTS
from mcts.searcher.mcts import MCTS
import copy,pickle,os,shutil

class LinkAction(BaseAction):
    def __init__(self, deltaState=None):
        #implement this
        pass
    def __hash__(self):
        #implement this
        pass
    def __eq__(self, other):
        #implement this
        pass

class LinkState(BaseState):
    def __init__(self, parent=None, action=None, *, K=5, B=10,
                 potential_rod_length=[float(i) for i in range(1,6)],
                 motor_rad_optimizable=False):
        self.state = copy.deepcopy(parent.state+action.deltaState) if parent is not None and action is not None else []
        self.anneal = LinkageAnnealer(K,B)
        self.potential_rod_length = potential_rod_length
        self.motor_rad_optimizable = motor_rad_optimizable
    def get_possible_actions(self) -> [any]:
        #implement this
        pass
    def take_action(self, action: any) -> 'BaseState':
        #implement this
        pass
    def is_terminal(self) -> bool:
        #implement this
        pass
    def get_reward(self) -> float:
        e=0
        #implement this
        print('experienced energy: %f'%e)
        self.write(e)
        return e
    def get_current_player(self) -> int:
        return 1
    def write(self, e):
        with open('mcts_results/e[%f].pickle'%e, 'wb') as handle:
            pickle.dump(self.anneal.state, handle)

if __name__=='__main__':
    if os.path.exists('mcts_results'):
        shutil.rmtree('mcts_results')
    if not os.path.exists('mcts_results'):
        os.mkdir('mcts_results')
    mcts = ParallelMCTS(iteration_limit=10000)
    mcts.search(LinkState(K=5))