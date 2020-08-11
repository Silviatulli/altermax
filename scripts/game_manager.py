#!/usr/bin/env python
import numpy as np
from minmax.game_model import GameState
from minmax import Q
from minmax.interface import View
from minmax.robot_decision import Robot
from minmax.child_decision import Child
# from robot_manager import RobotManager
import time
from copy import deepcopy


def play_game(robot, child):
    state = GameState()
    visualization = View()
    num_actions = 0

    while not state.isFinished():
        visualization.update(state)
        # time.sleep(0.5)

        valid_actions = state.valid_actions()

        if state.is_child_turn:
            action = child.policy(state)
            num_actions += 1

        else:
            action = robot.policy(state)
            robot.give_demonstration(action, state)

        old_state = deepcopy(state)
        print(GameState.get_state_id(state))
        state, reward, done, info = state.make_action(action)

        # reward function
        if old_state.get_score('child') < state.get_score('child'):
            reward = 1
        else:
            reward = -1
        child.update(old_state, action, reward, state)

    visualization.update(state)
    time.sleep(1.5)

    if state.is_child_turn and state.isFinished():
        outcome = 'Failure'
    elif not state.is_child_turn and state.isFinished():
        outcome = 'Success'

    return outcome, num_actions


if __name__ == "__main__":
    number_of_games = 3
    robot = Robot()
    child = Child()
    # robot = RobotManager()

    for game in range(number_of_games):
        outcome, num_actions = play_game(robot, child)
        robot.update_POMDP(outcome, num_actions)