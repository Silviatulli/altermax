from minmax.game_model import GameState
import cachetools


def lazy_copy(node):
    """
    this copy is meant to have a single move performed on it
    and otherwise shares as much data as possible with its parent
    """
    if node.is_child_turn:
        ball_dict_copy = {
            'robot': node.balls['robot'],
            'child': node.balls['child'].copy()
        }
    else:
        ball_dict_copy = {
            'robot': node.balls['robot'].copy(),
            'child': node.balls['child']
        }

    return Node(
        is_child_turn=node.is_child_turn,
        balls=ball_dict_copy
    )


class Node(GameState):
    __slots__ = ['balls', 'is_child_turn']

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
        # Note: Children are read-only
        nodes = list()

        for valid_action in self.valid_actions():
            env_copy = lazy_copy(self)
            env_copy.make_action(valid_action)
            nodes.append(env_copy)

        return nodes


@cachetools.cached(cache=cachetools.Cache(int(1e5)),
                   key=lambda n, d: (GameState.get_state_id(n), d))
def minimax(node, depth):
    score = node.score()
    isFinished = node.isFinished()
    maximizingPlayer = node.is_child_turn

    if depth == 0 or isFinished:
        return score

    children = node.children()

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


# @cachetools.cached(cache=cachetools.LRUCache(int(1e5)),
#                    key=lambda s, a: (GameState.get_state_id(s), a))
def Q(state, action):

    env_copy = lazy_copy(state)
    env_copy.make_action(action)
    q_value = V(env_copy)
    return q_value


if __name__ == "__main__":
    node = Node()
    value = minimax(node, 6)
    print(value)
