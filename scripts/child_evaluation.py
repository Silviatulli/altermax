# relative - comparing two policies - performance estimate of the child model
# import the modules: child decision and robot decision
# define a policy (q-learning)
# define optimal policy performance (performance metrics)
# compare number of win percentage for a x episodes (roll out policies
#  - compute the accumulated rewards - minmax child vs qlearning child)
# in this way we have a baseline for the number of games we need to
# play
# define train steps: set the number of episodes (how many times you want
# the child plays the game)
# play the game x times
# save the q values
# check how many times the q values are updated

from minmax.child_minmax import ChildMinmax
from minmax.child_qlearning import ChildQlearning
from minmax.robot_decision import Robot
from minmax.game_model import GameState
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.style
from multiprocessing import Pool

# TODO create child_minmax option


def play_game(robot, child, isTraining=True):
    state = GameState()
    num_actions = 0

    while not state.isFinished():

        valid_actions = state.valid_actions()

        if state.is_child_turn:
            action = child.policy(state)
            num_actions += 1

        else:
            action = robot.policy(state)

            if isTraining:
                (demonstration_state,
                 demonstration_action,
                 demonstration_reward,
                 demonstration_new_state) = robot.give_demonstration(action,
                                                                     state)
                child.demonstration_update(demonstration_state,
                                           demonstration_action,
                                           demonstration_reward,
                                           demonstration_new_state)

                # examples = robot.give_explanation()
                # child.explanation_update(examples)

        old_state_idx = GameState.get_state_id(state)
        old_score = state.get_score('child')
        state, reward, done, info = state.make_action(action)
        new_state_idx = GameState.get_state_id(state)
        if isTraining:
            # reward function
            if old_score < state.get_score('child'):
                reward = 1
            else:
                reward = -1
            child.update(old_state_idx, action, reward, new_state_idx)

    if state.is_child_turn and state.isFinished():
        outcome = -1
    elif not state.is_child_turn and state.isFinished():
        outcome = 1

    return outcome, num_actions


# keyword argument - optional or extra arguments


def train(robot, child, num_episodes=1):
    for episode in tqdm(range(num_episodes)):
        outcome, num_actions = play_game(robot, child)


def evaluate(robot, child):
    num_episodes = 50
    win = 0
    for episode in tqdm(range(num_episodes)):
        outcome, num_actions = play_game(robot,
                                         child,
                                         isTraining=False)
        if outcome == 1:
            win += 1
    win_rate_child = win * 1.0/num_episodes
    return win_rate_child

# TODO:
# print rewards for child_minmax and child_qlearning
# plot rewards per time steps


def process(robot, child):
    episodes_between_evaluations = 100
    minimaxChild = ChildMinmax()
    threshold = evaluate(robot, minimaxChild)
    games_played = 0
    win_rate_child = evaluate(robot, child)
    while win_rate_child < threshold or games_played < 1000:
        train(robot, child, num_episodes=episodes_between_evaluations)
        games_played += episodes_between_evaluations
        win_rate_child = evaluate(robot, child)
        print("QLearningChild performance: {0}".format(win_rate_child))

    return games_played


if __name__ == "__main__":
    for _ in tqdm(range(2)):
        process(Robot(), ChildQlearning())

    # with Pool(processes=5) as pool:
    #     game_tuples = [(Robot(), ChildQlearning())] * 5
    #     performance_list = pool.starmap(process, game_tuples)
    #     average_performance = sum(performance_list)/len(performance_list)
    #     msg_average = ("QLearning need {0} episodes on average",
    #                    " to be as good as the minmax.")
    #     print(msg_average.format(average_performance))

    # visualize performance list
    # create joint plot

    # for game in range(number_of_games):
    #     outcome, num_actions = play_game(robot,child_qlearning)
    #     robot.update_POMDP(outcome, num_actions)
