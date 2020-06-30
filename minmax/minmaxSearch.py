from minmax.game_model import GameState
import functools
from copy import deepcopy


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

        # populate tree
        for valid_action in self.valid_actions():
            env_copy = deepcopy(self)
            env_copy.make_action(valid_action)
            # node = Node(env_copy.is_child_turn, env_copy.balls)
            nodes.append(env_copy)

        return nodes


@functools.lru_cache(maxsize=int(5e5), typed=False)
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


@functools.lru_cache(maxsize=int(5e5), typed=False)
def Q(state, action):
    action = action
    env_copy = deepcopy(state)
    env_copy.make_action(action)
    q_value = V(env_copy)
    return q_value


if __name__ == "__main__":
    node = Node()
    value = minimax(node, 6)
    print(value)
