from mcts.searcher.mcts import TreeNode,MCTS
import concurrent.futures
import multiprocessing

class ParallelMCTS(MCTS):
    def __init__(self, nProcess=multiprocessing.cpu_count()//2,
                 time_limit: int = None,
                 iteration_limit: int = None,
                 exploration_constant: float = None,
                 rollout_policy=None):
        super().__init__(time_limit=time_limit,
                         iteration_limit=iteration_limit,
                         exploration_constant=exploration_constant,
                         rollout_policy=rollout_policy)
        self.nProcess=nProcess
    def execute_round(self):
        """
            execute a selection-expansion-simulation-backpropagation round
        """
        #serial selection
        nodes=[]
        for i in range(self.nProcess):
            nodes.append(self.select_node(self.root))
        for i in range(self.nProcess):
            parent = nodes[i].parent
            parent.children[nodes[i].action] = nodes[i]
            if len(parent.children) == len(parent.state.get_possible_actions()):
                parent.is_fully_expanded = True

        #parallel reward computation
        tasks=[]
        rewards=[]
        executor = concurrent.futures.ProcessPoolExecutor(max_workers=self.nProcess)
        for i in range(self.nProcess):
            tasks.append(executor.submit(self.rollout_policy, nodes[i].state))
        executor.shutdown(wait=True)
        for i in range(self.nProcess):
            rewards.append(tasks[i].result())

        #serial back-prop
        for i in range(self.nProcess):
            self.backpropogate(nodes[i], rewards[i])
    def expand(self, node: TreeNode) -> TreeNode:
        actions = node.state.get_possible_actions()
        for action in actions:
            if action not in node.children:
                newNode = TreeNode(node.state.take_action(action), node)
                newNode.action = action
                #node.children[action] = newNode    #we need to delay this until after parallel backpropagation
                #if len(actions) == len(node.children):
                #    node.is_fully_expanded = True
                return newNode

        raise Exception("Should never reach here")
