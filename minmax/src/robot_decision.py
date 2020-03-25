#!/usr/bin/env python
import numpy as np
from game_model import GameState
from minmax import Q

class Robot(object):
    #TODO: action selection - policy
    #get child action and update POMDP
    #generate an explanation

    def policy(self,state):
        #given the current state of the game makes the robot selects an action in the game
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


    def update_POMDP(self,outcome,num_actions):
        
        #changes the POMPD based on the child actions
        
        return 

    def generate_explanation(self, action, state):
        #given the current state of the child makes the robot selects an action/explanation
        #count number of turns
        return 'this is a random explanation'


if __name__ == "__main__":
    pass
 