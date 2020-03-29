
import numpy as np
from game_model import GameState
from minmax import Q



class Child(object):
    #TODO: 
    #      Representation of relevant data:
    #      q table
    #
    #      Things your code needs to be able to do:(features)
    #      ability to update the child
    #      perform an action

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
    
    def update(self, state, action, new_state):
        # reward function
        if state.get_score('child') < new_state.get_score('child'):
            reward = 1
        else:
            reward = -1
        return

        

if __name__ == "__main__":
    pass