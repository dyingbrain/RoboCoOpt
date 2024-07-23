from mcts.base.base import BaseAction, BaseState
from robocoopt.opt.anneal.optimizer_anneal import LinkageAnnealer
from robocoopt.opt.mcts.parallel_mcts import ParallelMCTS
from mcts.searcher.mcts import MCTS
import copy, pickle, os, shutil, random, math

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
                 potential_rod_length=[float(i) for i in range(1, 6)],
                 motor_rad_optimizable=False):
        self.state = copy.deepcopy(parent.state + action.deltaState) if parent is not None and action is not None else []
        self.anneal = LinkageAnnealer(K, B)
        self.potential_rod_length = potential_rod_length
        self.motor_rad_optimizable = motor_rad_optimizable

    def get_possible_actions(self) -> [LinkAction]:
        actions = []
        current_node_index = len(self.state) // 5

        if current_node_index < self.anneal.K:
            for state in [LinkageAnnealer.MOVABLE, LinkageAnnealer.FIXED]:
                for c1 in range(current_node_index):
                    for c2 in range(current_node_index):
                        if c1 != c2:
                            for len1 in self.potential_rod_length:
                                for len2 in self.potential_rod_length:
                                    if state == LinkageAnnealer.FIXED:
                                        x = random.uniform(-self.anneal.B, self.anneal.B)
                                        y = random.uniform(-self.anneal.B, self.anneal.B)
                                        actions.append(LinkAction([c1, c2, len1, len2, state, x, y]))
                                    else:
                                        actions.append(LinkAction([c1, c2, len1, len2, state]))

     
        if self.motor_rad_optimizable and current_node_index == 0:
            for rad_delta in [-0.1, 0.1, -0.2, 0.2]:
                actions.append(LinkAction([None, None, None, None, None, rad_delta]))

      
        actions.append(LinkAction([LinkageAnnealer.NOT_USED, LinkageAnnealer.NOT_USED, 0.0, 0.0,
                                   LinkageAnnealer.NOT_USED]))

        return actions

    def take_action(self, action: any) -> 'BaseState':
        new_state = LinkState(parent=self, action=action, K=self.anneal.K, B=self.anneal.B,
                             potential_rod_length=self.potential_rod_length,
                             motor_rad_optimizable=self.motor_rad_optimizable)
        
 
        if not new_state.is_geometrically_feasible():
            return self 

        return new_state 

    def is_terminal(self) -> bool:
        return len(self.state) > 0 and self.state[-1] == LinkageAnnealer.NOT_USED

    def get_reward(self) -> float:
        self.anneal.state = copy.deepcopy(self.state)
        while self.anneal.state[-1] == LinkageAnnealer.NOT_USED:
            self.anneal.state = self.anneal.state[0:-5]
        if not self.anneal.check_topology_feasibility():
            return -1
        if not self.anneal.check_geometry_feasibility():
            return -1
        e = -self.anneal.energy()
        print('experienced energy: %f' % e)
        self.write(e)
        return e

    def get_current_player(self) -> int:
        return 1

    def write(self, e):
        with open('mcts_results/e[%f].pickle' % e, 'wb') as handle:
            pickle.dump(self.anneal.state, handle)

    def is_geometrically_feasible(self):
        self.anneal.state = copy.deepcopy(self.state)
        
        while self.anneal.state and self.anneal.state[-1] == LinkageAnnealer.NOT_USED:  
            self.anneal.state = self.anneal.state[0:-5]
        
        link = self.anneal.set_to_linkage()
        return link.check_geometry_feasibility()


if __name__ == '__main__':
    if os.path.exists('mcts_results'):
        shutil.rmtree('mcts_results')
    if not os.path.exists('mcts_results'):
        os.mkdir('mcts_results')
    mcts = ParallelMCTS(iteration_limit=10000)
    mcts.search(LinkState(K=5))