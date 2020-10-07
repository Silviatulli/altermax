from minmax.minmaxSearch import minimax
from copy import deepcopy
from minihex import player, HexGame
from random import randint


class Node(object):
    def __init__(self, sim):
        self.sim = sim
        self.is_child_turn = self.sim.active_player == self.sim.player

    def score(self):
        # match the reward function of Tug of War
        if not self.sim.done:
            return 0

        if self.sim.winner is None:
            return 0
        elif self.sim.winner == self.sim.player:
            return 1
        else:
            return -1

    def isFinished(self):
        return self.sim.done

    def children(self):
        actions = self.sim.get_possible_actions()

        children = list()
        for action in actions:
            new_sim = deepcopy(self.sim)
            new_sim.make_move(action)
            children.append(Node(new_sim))

        return children


class MinMaxAgent(object):
    def __init__(self, config, *, player=player.BLACK, debug=False):
        self.player = player
        self.debug = debug
        self.depth = int(config["MinMaxAgent"]["search_depth"])

    def act(self, board, player, info):
        sim = HexGame(
            active_player=player,
            board=board,
            focus_player=self.player,
            debug=self.debug
        )

        actions = sim.get_possible_actions()
        next_actions = list()
        next_score = None
        for action in actions:
            tmp_sim = deepcopy(sim)
            tmp_sim.make_move(action)
            root_node = Node(tmp_sim)

            score = minimax(root_node, self.depth)
            print(f"Score for action {action} is: {score}")

            if next_score is None or next_score < score:
                next_actions = [action]
                next_score = score
            elif next_score == score:
                next_actions.append(action)

        idx = randint(0, len(next_actions) - 1)
        next_action = next_actions[idx]

        return next_action

    def explain(self, state, action):
        pass
