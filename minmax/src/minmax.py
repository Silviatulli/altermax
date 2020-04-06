from game_model import GameState
import functools

class Node(GameState):
    def __init__(self, is_child_turn=None, balls=None):
        if is_child_turn is None:
            super(Node, self).__init__()
        else:
            self.is_child_turn = is_child_turn
            self.balls = balls
            
    def score(self):
        if not self.is_child_turn and self.isFinished():
            return 1
        elif self.is_child_turn and self.isFinished():
            return -1
        else:
            return 0
    
    def children(self):
        nodes = list()

        #populate tree
        for valid_action in self.valid_actions():
            new_state = self.make_action(valid_action[0], valid_action[1])
            node = Node(new_state.is_child_turn, new_state.balls)
            nodes.append(node)
            
        return nodes

@functools.lru_cache(maxsize=None, typed=False)
def minimax(node, depth):
    score = node.score()
    children = node.children()
    isFinished = node.isFinished()
    maximizingPlayer = node.is_child_turn

    if depth == 0 or isFinished:
        return score

    if maximizingPlayer:
        value = -1
        for child in children:
            value = max(value, minimax(child, depth - 1))
        return value
    else:
        value = 1
        for child in children:
            value = min(value, minimax(child, depth - 1))
        return value

def V(state):
    node = Node(is_child_turn=state.is_child_turn, balls=state.balls)
    value = minimax(node, 3)
    return value

@functools.lru_cache(maxsize=None, typed=False)
def Q(state, action):
    action, ball_id = action
    new_state = state.make_action(action, ball_id)
    q_value = V(new_state)
    return q_value
    
if __name__ == "__main__":
    node = Node()
    value = minimax(node, 3)
    print(value)