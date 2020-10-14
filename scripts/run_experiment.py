from minmax.child_minmax import ChildMinmax
from minmax.child_qlearning import ChildQlearning
from minmax.robot_decision import Robot
from minmax.game_model import GameState
from tqdm import tqdm
import numpy as np
from multiprocessing import Pool
import configparser
import pandas as pd
from copy import deepcopy
from itertools import count


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
                    demonstration = robot.give_demonstration(action,
                                                             state)

                    child.demonstration_update(*demonstration)

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
        outcome = 0
    elif not state.is_child_turn and state.isFinished():
        outcome = 1
    return outcome, metrics


def evaluate(robot, child, config):
    num_episodes = int(config["Evaluation"]["Episodes"])
    win = 0

    for episode in range(num_episodes):
        outcome, num_actions = play_game(robot,
                                         child,
                                         isTraining=False)
        if outcome == 1:
            win += 1
    win_rate_child = win * 1.0 / num_episodes

    return win_rate_child


def process(task):
    robot, child, config = task
    num_episodes = int(config["Training"]["Episodes"])
    condition = int(config["General"]["Condition"])
    trial = int(config["General"]["trail_idx"])
    use_demonstrations = config["Training"]["use_demonstrations"] == "True"
    use_explanations = config["Training"]["use_explanations"] == "True"
    filter_length = int(config["Training"]["RollingAvgFilterLength"])
    threshold = float(config["Evaluation"]["threshold"])
    max_samples = int(config["General"]["max_samples"])

    result_rows = list()
    wins_log = list()

    total_examples = 0
    episode = 0
    while total_examples < max_samples:
        outcome, metrics = play_game(robot, child, metrics=None,
                                     use_demonstrations=use_demonstrations,
                                     use_explanations=use_explanations)

        wins_log.append(outcome)
        low = max([0, len(wins_log) - filter_length])
        rolling_winrate = sum(wins_log[low:]) / filter_length

        total_examples += metrics["total_experience"]
        result_rows.append((condition, trial, episode,
                            total_examples, rolling_winrate, outcome))

        episode += 1
        if rolling_winrate > threshold:
            break

    return result_rows


if __name__ == "__main__":
    config = configparser.ConfigParser()
    config.read('scripts/TugOfWar.ini')

    minimaxChild = ChildMinmax()
    threshold = evaluate(Robot(), minimaxChild, config)
    config["Evaluation"]["threshold"] = str(threshold)

    total_rows = 0
    dataframe = pd.DataFrame(columns=["Condition", "Trial", "Episode",
                                      "Total Examples", "Winrate", "Result"])

    trials = int(config["General"]["trials"])
    conditions = [
        {"use_demonstrations": False, "use_explanations": False},
        {"use_demonstrations": True, "use_explanations": False},
        {"use_demonstrations": False, "use_explanations": True},
        {"use_demonstrations": True, "use_explanations": True},
    ]

    result_logs = dict()

    if config["General"]["use_multiprocessing"] == "True":
        num_workers = int(config["General"]["num_workers"])
        pool = Pool(processes=num_workers)

    tasks = list()
    for condition_idx, condition in enumerate(conditions):
        result_logs[condition_idx] = {
            "total_examples": list(),
            "winrate": list()
        }

        for trial_idx in range(trials):
            local_config = deepcopy(config)
            local_config["Training"]["use_demonstrations"] = str(
                condition["use_demonstrations"])
            local_config["Training"]["use_explanations"] = str(
                condition["use_explanations"])
            local_config["General"]["Condition"] = str(condition_idx)
            local_config["General"]["trail_idx"] = str(trial_idx)
            tasks += [(Robot(), ChildQlearning(), local_config)]

    if config["General"]["use_multiprocessing"] == "True":
        iterable = pool.imap_unordered(process, tasks)
    else:
        iterable = map(process, tasks)

    pbar = tqdm(iterable, desc="Trials", total=len(tasks))
    for result in pbar:
        for row in result:
            dataframe.loc[total_rows] = row
            total_rows += 1

    if config["General"]["use_multiprocessing"] == "True":
        pool.close()

    file_name = config["Evaluation"]["result_file"]
    dataframe.to_excel(file_name)
