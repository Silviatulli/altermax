#!/usr/bin/env python
import numpy as np
from minimax import GameState
from minmax import Q


class ChildMinmax(object):

    def policy(self, state):
        # given the current state of the game makes the child selects an
        # action in the game
        valid_actions = state.valid_actions()
        best_action = valid_actions[0]
        max_value = Q(state, best_action)
        for action in valid_actions:
            if max_value < Q(state, action):
                max_value = Q(state, action)
                best_action = action

        best_actions = list()
        for action in valid_actions:
            if max_value == Q(state, action):
                best_actions.append(action)

        idx = np.random.randint(len(best_actions))
        return best_actions[idx]

    def update(self, state, action, new_state):
        return

    def demonstration_update(self, state, action, reward, new_state):
        return

    def explanation_update(self, examples):
        return


if __name__ == "__main__":
    pass
