from minmax.child_minmax import ChildMinmax
from minmax.child_qlearning import ChildQlearning
from minmax.robot_decision import Robot
from minmax.game_model import GameState
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from multiprocessing import Pool, cpu_count

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

                examples = robot.give_examples()
                child.examples_update(examples)

                #other_actions = robot.give_other_actions(action,state)
                #child.other_actions_update(other_actions)


        old_state_idx = GameState.get_state_id(state)
        old_score = state.get_score('child')
        state, reward, done, info = state.make_action(action)
        new_state_idx = GameState.get_state_id(state)
        

        if isTraining:
            reward = old_score - state.get_score('child')   
            child.update(old_state_idx, action, reward, new_state_idx)
        
            
    if state.is_child_turn and state.isFinished():
        outcome = -1
    elif not state.is_child_turn and state.isFinished():
        outcome = 1
    return outcome, num_actions


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
    
<<<<<<< HEAD
=======
    #other_actions_matrix = robot.other_actions
    #plt.imshow(other_actions_matrix)
    #plt.show()
>>>>>>> c06f2e42171a6e9519228198f1f9fa0f0f81fdf7

    return games_played


if __name__ == "__main__":
    with Pool(processes=cpu_count() - 2) as pool:
        game_tuples = [(Robot(), ChildQlearning())] * 5
        performance_list = pool.starmap(process, game_tuples)

        average_performance = sum(performance_list)/len(performance_list)
        msg_average = ("QLearning need {0} episodes on average"
                       " to be as good as the minmax.")
        print(msg_average.format(average_performance))


