#!/usr/bin/env python
import numpy as np
from minmax import GameState
from minmax.minmaxSearch import Q
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.style
import json
<<<<<<< HEAD
=======

>>>>>>> c06f2e42171a6e9519228198f1f9fa0f0f81fdf7

class Robot(object):

    def policy(self, state):
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

    def update_POMDP(self, outcome, num_actions):
        return

    def give_demonstration(self, action, state):
        old_score = state.get_score('child')
        new_state = deepcopy(state)
        new_state.make_action(action)

        reward = old_score - state.get_score('child')

        return (state, action, reward, new_state)


    def give_other_actions(self, action, state):
        valid_actions = state.valid_actions()
        scores = []
        for iter_action in valid_actions:
            current_state = deepcopy(state)
            current_state.make_action(iter_action)
            score = current_state.get_score('robot')
            scores.append(score)

        action_idx = valid_actions.index(action)
        current_score = scores.pop(action_idx)
        valid_actions.pop(action_idx)

        scores = np.asarray(scores)
        valid_actions = np.asarray(valid_actions)

        if valid_actions.size > 3:
            action_idx = np.random.choice(valid_actions.size, size=3, replace=False)
            other_actions = valid_actions[action_idx]
            scores = scores[action_idx]
        elif valid_actions.size == 0:
            return np.array([])
        else:
            other_actions = valid_actions


        scores = np.array(scores)
        rewards = current_score - scores
        order = np.argsort(rewards)
        other_actions = other_actions[order]
        rewards = rewards[order]
        other_actions_matrix = np.column_stack((other_actions, rewards))
        return  other_actions_matrix


    def give_text(self, action, state):
        valid_actions = state.valid_actions()
        scores = []
        for iter_action in valid_actions:
            current_state = deepcopy(state)
            current_state.make_action(iter_action)
            score = current_state.get_score('robot')
            scores.append(score)

        action_idx = valid_actions.index(action)
        current_score = scores.pop(action_idx)
        valid_actions.pop(action_idx)

        scores = np.asarray(scores)
        valid_actions = np.asarray(valid_actions)

        if valid_actions.size > 3:
            action_idx = np.random.choice(valid_actions.size, size=3, replace=False)
            other_actions = valid_actions[action_idx]
            scores = scores[action_idx]
        elif valid_actions.size == 0:
            return np.array([])
        else:
            other_actions = valid_actions


        scores = np.array(scores)
        rewards = current_score - scores
        order = np.argsort(rewards)
        other_actions = other_actions[order]
        rewards = rewards[order]
        other_actions_matrix = np.column_stack((other_actions, rewards))

        return state, current_score, scores, action, other_actions, rewards

    def give_examples(self):
        array_shape = 12*12*12*12*2
        valid_states = []
        num_examples = 10
        while len(valid_states) < num_examples:
            #print('generating samples')
            candidate_states = np.random.randint(low=1,
                                                 high=array_shape,
                                                 size=10)
            for state_idx in candidate_states:
                state = GameState.get_state(state_idx)
                if (not state.is_child_turn
                        and state.isValid()
                        and not state.isFinished()
                        and len(state.valid_actions()) > 0):
                    valid_states.append(state_idx)

        valid_states = valid_states[:num_examples]

        examples = []
        for robot_state_idx in valid_states:
            robot_state = GameState.get_state(robot_state_idx)
            best_action = self.policy(robot_state)
            new_state, reward, done, info = robot_state.make_action(best_action)
            reward = robot_state.get_score('child') - new_state.get_score('child')
            example = (robot_state, best_action, reward, new_state)
            examples.append(example)
        

        #json_file = json.dumps(str(examples))

        #with open("json_file.json", "w") as file:
            #file.write(json_file)

        return examples

