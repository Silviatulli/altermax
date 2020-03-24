#!/usr/bin/env python
import numpy as np
from game_model import GameState
from minmax import Q
from interface import View
import time


def play_game():
    state = GameState()
    visualization = View()

    while not state.isFinished():
        visualization.update(state)
        time.sleep(0.5)
        
        valid_actions = state.valid_actions()

        if state.is_child_turn:
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
            action, ball_id = best_actions[idx]

        else:
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
            action, ball_id = best_actions[idx]

            #random_action
            #worst_action

        state = state.make_action(action, ball_id)
    visualization.update(state)
    time.sleep(1.5)

if __name__ == "__main__":
    number_of_games = 3
    for game in range(number_of_games):
        play_game()


