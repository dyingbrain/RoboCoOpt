from mcts.base.base import BaseAction,BaseState
from optimizer_anneal import LinkageAnnealer
from parallel_mcts import ParallelMCTS
from mcts.searcher.mcts import MCTS
import copy,pickle,os,shutil

class LinkAction(BaseAction):
    def __init__(self, deltaState=None):
        self.deltaState = deltaState
    def __hash__(self):
        return hash(tuple(self.deltaState))
    def __eq__(self, other):
        return self.deltaState[0] == other.deltaState[0] and\
        self.deltaState[1] == other.deltaState[1] and\
        self.deltaState[2] == other.deltaState[2] and\
        self.deltaState[3] == other.deltaState[3] and\
        self.deltaState[4] == other.deltaState[4]

class LinkState(BaseState):
    def __init__(self, parent=None, action=None, *, K=5, B=10,
                 potential_rod_length=[float(i) for i in range(1,6)],
                 motor_rad_optimizable=False):
        self.state = copy.deepcopy(parent.state+action.deltaState) if parent is not None and action is not None else []
        self.anneal = LinkageAnnealer(K,B)
        self.potential_rod_length = potential_rod_length
        self.motor_rad_optimizable = motor_rad_optimizable
    def get_possible_actions(self) -> [any]:
        pass
    def take_action(self, action: any) -> 'BaseState':
        return LinkState(self, action)
    def is_terminal(self) -> bool:
        return len(self.state)>0 and self.state[-1]==LinkageAnnealer.NOT_USED
    def get_reward(self) -> float:
        self.anneal.state=copy.deepcopy(self.state)
        while self.anneal.state[-1]==LinkageAnnealer.NOT_USED:
            self.anneal.state=self.anneal.state[0:-5]
        if not self.anneal.check_topology_feasibility():
            return -1
        if not self.anneal.check_geometry_feasibility():
            return -1
        e=-self.anneal.energy()
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