import numpy as np
import random
from minmax import GameState, Action


class ChildQlearning(object):
    def __init__(self):
        num_states = (12 * 12)**2 * 2
        self.q_table = np.zeros((num_states, len(Action)))

    def Q(self, state, action):
        q_value = self.q_table[state, action]

        return q_value

    def policy(self, state):
        state_idx = GameState.get_state_id(state)
        valid_actions = state.valid_actions()
        best_action = valid_actions[0]
        max_value = self.Q(state_idx, best_action)
        epsilon = 0.15
        for action in valid_actions:
            if max_value < self.Q(state_idx, action):
                max_value = self.Q(state_idx, action)
                best_action = action

        best_actions = list()
        for action in valid_actions:
            if max_value == self.Q(state_idx, action):
                best_actions.append(action)

        if np.random.rand() <= epsilon:
            idx = np.random.randint(len(valid_actions))
            action = valid_actions[idx]
        else:
            idx = np.random.randint(len(best_actions))
            action = best_actions[idx]

        return action

    def update(self, state, action, reward, new_state):
        alpha = 0.8
        gamma = 0.99
        V_star = np.max(self.q_table[new_state, :])
        q_sa = self.Q(state, action)
        q_value = (1 - alpha) * q_sa + alpha * (reward + gamma * V_star)

        self.q_table[state, action] = q_value

    def demonstration_update(self, state, action, reward, new_state):
        new_state_idx = GameState.get_state_id(new_state)
        state_idx = GameState.get_state_id(state)
        alpha = 0.8
        gamma = 0.99
        q_value = (1 - alpha) * self.Q(state_idx, action) + alpha * \
            (reward + gamma * np.max(self.q_table[new_state_idx, :]))

        self.q_table[state_idx, action] = q_value

    def examples_update(self, examples):
        for example in examples:
            self.update(*example)

    def explanation_update(self, explanations):
        for explanation in explanations:
            (state_idx, _, _, reward,
             alternative_action, alternative_state_idx,
             score_difference) = explanation
            self.update(
                state_idx,
                alternative_action,
                score_difference + reward,
                alternative_state_idx
            )
