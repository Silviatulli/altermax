#!/usr/bin/env python
import numpy as np
from minmax import GameState
from minmax.minmaxSearch import Q
from copy import deepcopy


class Robot(object):
    # TODO: action selection - policy
    # get child action and update POMDP
    # generate an explanation

    def policy(self, state):
        # given the current state of the game makes the robot selects
        # an action in the game
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
        # changes the POMPD based on the child actions
        return

    def give_demonstration(self, action, state):

        # given the current state of the child makes the robot selects
        # an action/explanation count number of turns

        # TODO: transform the explanation into a demonstration
        # consider state, action and reward
        # reward function - modify here and in the child_evaluation


        # make the reward function more sophisticated
        # from the current state, perform an action and store the reward
        current_state = deepcopy(state)
        current_state.make_action(action)
        rewards = []
        if state.get_score('child') < old_score:
            reward = 1
        else:
            reward = -1
        
        rewards.append(reward)

        # use the minmax to compute the reward that you could get from performing a bunch of other actions (3(?))
        
        # evaluate the goodness of the performed action giving a reward with respect to the other action outputs




        old_score = state.get_score('child')
        new_state = deepcopy(state)
        new_state.make_action(action)

        if state.get_score('child') < old_score:
            reward = 1
        else:
            reward = -1

        return (state, action, reward, new_state)


    def give_other_actions(self, action, state):

        # make the reward function more sophisticated
        # from the current state, perform an action and store the score
        current_state = deepcopy(state)
        current_state.make_action(action)
        current_score = state.get_score('child')

        # use the minmax to compute the score that you could get from performing a bunch of other actions (3(?))
        # evaluate the goodness of the performed action giving a reward with respect to the other action outputs
        valid_actions = state.valid_actions()
        action_idx = np.randint(len(valid_actions), shape=(3,))
        other_actions = valid_actions[action_idx]
        scores = []
        for other_action in other_actions:
            current_state = deepcopy(state)
            current_state.make_action(other_action)
            score = current_state.get_score('child')
            scores.append(score)

        # TODO: generate a NL string in the form of "If I would take action A instead of action B I would get this amount ---"
        # check f-string
        # 


        score = np.array(score)
        rewards = current_score - score
        order = np.argsort(rewards)
        other_actions = other_actions[order]
        rewards = rewards[order]
        other_actions_matrix = np.column_stack((other_action, rewards))
        print(other_action, rewards)

        return  other_actions_matrix


    def give_explanation(self):
        array_shape = 12*12*12*12*2

        valid_states = []
        num_examples = 10
        while len(valid_states) < num_examples:
            print('generating samples')
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

            if robot_state.get_score('child') < new_state.get_score('child'):
                reward = 1
            else:
                reward = -1

            example = (robot_state, best_action, reward, new_state)
            examples.append(example)

        return examples
