#!/usr/bin/env python
import numpy as np
from game_model import GameState
from minmax import Q
from interface import View
from robot_decision import Robot
from child_decision import Child
#from robot_manager import RobotManager
import time


def play_game(robot,child):
    state = GameState()
    visualization = View()
    num_actions = 0

    while not state.isFinished():
        visualization.update(state)
        #time.sleep(0.5)
        
        valid_actions = state.valid_actions()
        
        if state.is_child_turn:
            action, ball_id = child.policy(state)
            num_actions += 1


        else:
            action, ball_id = robot.policy(state)
            robot.give_explanation((action, ball_id), state)

            #random_action
            #worst_action

        old_state = state
        state = state.make_action(action, ball_id)
        child.update(old_state, (action, ball_id), state)

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
    child = Child()
    # robot = RobotManager()

    for game in range(number_of_games):
        outcome, num_actions = play_game(robot,child)
        robot.update_POMDP(outcome, num_actions)
        
