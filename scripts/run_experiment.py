from minmax.child_minmax import ChildMinmax
from minmax.child_qlearning import ChildQlearning
from minmax.robot_decision import Robot
from minmax.game_model import GameState
from tqdm import tqdm
import numpy as np
from multiprocessing import Pool, cpu_count, current_process
import configparser
import json
import pandas as pd


def play_game(robot, child, *, isTraining=True,
              use_demonstrations=False, use_explanations=False,
              metrics=None):
    state = GameState()
    if metrics is None:
        metrics = {
            "total_explanations": 0,
            "total_demonstrations": 0,
            "total_actions": 0,
            "total_experience": 0
        }
    while not state.isFinished():
        valid_actions = state.valid_actions()

        if state.is_child_turn:
            action = child.policy(state)
        else:
            action = robot.policy(state)

            if isTraining:
                if use_demonstrations:
                    (demonstration_state,
                     demonstration_action,
                     demonstration_reward,
                     demonstration_new_state) = robot.give_demonstration(action,
                                                                         state)

                    child.demonstration_update(demonstration_state,
                                               demonstration_action,
                                               demonstration_reward,
                                               demonstration_new_state)

                    metrics["total_demonstrations"] += 1
                    metrics["total_experience"] += 1

                if use_explanations:
                    examples = robot.give_examples()
                    child.examples_update(examples)
                    metrics["total_explanations"] += len(examples)
                    metrics["total_experience"] += len(examples)

        old_state_idx = GameState.get_state_id(state)
        old_score = state.get_score('child')
        state, reward, done, info = state.make_action(action)
        new_state_idx = GameState.get_state_id(state)

        if isTraining:
            reward = old_score - state.get_score('child')
            child.update(old_state_idx, action, reward, new_state_idx)
            metrics["total_actions"] += 1
            metrics["total_experience"] += 1

    if state.is_child_turn and state.isFinished():
        outcome = -1
    elif not state.is_child_turn and state.isFinished():
        outcome = 1
    return outcome, metrics


def train(robot, child, config, *, metrics=None):
    num_episodes = int(config["Training"]["Episodes"])
    use_demonstrations = config["Training"]["use_demonstrations"] == "True"
    use_explanations = config["Training"]["use_explanations"] == "True"

    try:
        pid = current_process()._identity[0]
    except IndexError:
        pid = 1
    pbar = tqdm(range(num_episodes), desc=f"Thread {pid} - Training",
                total=num_episodes, leave=False, position=pid)

    for episode in pbar:
        _, metrics = play_game(robot, child, metrics=metrics,
                               use_demonstrations=use_demonstrations,
                               use_explanations=use_explanations)
    return metrics


def evaluate(robot, child, config):
    num_episodes = int(config["Evaluation"]["Episodes"])
    win = 0

    try:
        pid = current_process()._identity[0]
    except IndexError:
        pid = 1
    pbar = tqdm(range(num_episodes), desc=f"Thread {pid} - Evaluation",
                total=num_episodes, leave=False, position=pid)

    for episode in pbar:
        outcome, num_actions = play_game(robot,
                                         child,
                                         isTraining=False)
        if outcome == 1:
            win += 1
    win_rate_child = win * 1.0 / num_episodes

    return win_rate_child


def process(robot, child, config):
    num_episodes = int(config["Training"]["Episodes"])
    max_games = int(config["General"]["max_games"])

    experience_log = list()
    winrate_log = list()

    minimaxChild = ChildMinmax()
    threshold = evaluate(robot, minimaxChild, config)

    win_rate_child = evaluate(robot, child, config)
    experience_log.append(0)
    winrate_log.append(win_rate_child)

    games_played = 0
    while win_rate_child < threshold or games_played < max_games:
        metrics = train(robot, child, config)

        games_played += num_episodes

        win_rate_child = evaluate(robot, child, config)
        experience_log.append(experience_log[-1] + metrics["total_experience"])
        winrate_log.append(win_rate_child)

    return (experience_log, winrate_log)


if __name__ == "__main__":
    config = configparser.ConfigParser()
    config.read('scripts/TugOfWar.ini')

    total_rows = 0
    dataframe = pd.DataFrame(columns=["Condition", "Trial",
                                      "Total Examples", "Winrate"])

    trials = int(config["General"]["trials"])
    conditions = [
        {"use_demonstrations": False, "use_explanations": False},
        {"use_demonstrations": True, "use_explanations": False},
        {"use_demonstrations": False, "use_explanations": True},
        {"use_demonstrations": True, "use_explanations": True},
    ]

    result_logs = dict()

    if config["General"]["use_multiprocessing"] == "True":
        pool = Pool(processes=cpu_count() - 2)

    pbar = tqdm(iter(conditions), desc="Condition", total=len(conditions))
    for condition_idx, condition in enumerate(pbar):
        result_logs[condition_idx] = {
            "total_examples": list(),
            "winrate": list()
        }
        config["Training"]["use_demonstrations"] = str(
            condition["use_demonstrations"])
        config["Training"]["use_explanations"] = str(
            condition["use_explanations"])

        game_tuples = [(Robot(), ChildQlearning(), config)] * trials
        if config["General"]["use_multiprocessing"] == "True":
            performance_list = pool.starmap(process, game_tuples)
        else:
            performance_list = list()
            for item in game_tuples:
                performance_list.append(process(*item))

        for trial_idx, performance in enumerate(performance_list):
            experience_log, winrate_log = performance
            for exp, winrate in zip(experience_log, winrate_log):
                row = (condition_idx, trial_idx, exp, winrate)
                dataframe.loc[total_rows] = row
                total_rows += 1

        pbar.update()

    if config["General"]["use_multiprocessing"] == "True":
        pool.close()

    file_name = config["Evaluation"]["result_file"]
    dataframe.to_excel(file_name)
