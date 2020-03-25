#!/usr/bin/env python
import numpy as np
from game_model import GameState
from minmax import Q
from interface import View
from robot_decision import Robot
import time


def play_game(robot):
    state = GameState()
    visualization = View()
    num_actions = 0

    while not state.isFinished():
        visualization.update(state)
        #time.sleep(0.5)
        
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
            num_actions += 1

        else:
            action, ball_id = robot.policy(state)
            explanation = robot.generate_explanation((action, ball_id), state)
            print(explanation)

            #random_action
            #worst_action

        state = state.make_action(action, ball_id)
    visualization.update(state)
    time.sleep(1.5)

    if state.is_child_turn and state.isFinished():
        outcome = 'Failure'
    elif not state.is_child_turn and state.isFinished():
        outcome = 'Success'
    
    print(outcome)
    print(num_actions)

    return outcome, num_actions

if __name__ == "__main__":
    number_of_games = 3
    robot = Robot()

    for game in range(number_of_games):
        outcome, num_actions = play_game(robot)
        robot.update_POMDP(outcome, num_actions)




