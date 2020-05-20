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
        try:
            best_action = valid_actions[0]
        except:
            #import pdb; pdb.set_trace()
            raise

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

    def give_demonstration(self, action, state):

        # given the current state of the child makes the robot selects an action/explanation
        # count number of turns

        # TODO: transform the explanation into a demonstration
        # consider state, action and reward

        # reward function
        new_state = state.make_action(action[0], action[1])
        if state.get_score('child') < new_state.get_score('child'):
            reward = 1
        else:
            reward = -1

        return (state, action, reward, new_state)
    
    def give_explanation(self):
        array_shape = 12*12*12*12*2 

        valid_states = []
        num_examples = 10
        while len(valid_states) < num_examples:
            candidate_states = np.random.randint(low=1, high=array_shape, size=10)
            for state_idx in candidate_states:
                state = GameState.get_state(state_idx)                
                if (state.is_child_turn == False 
                    and state.isValid() 
                    and not state.isFinished()
                    and len(state.valid_actions()) > 0):
                    valid_states.append(state_idx)
    
        valid_states = valid_states[:num_examples]
        
        examples = []
        for robot_state_idx in valid_states:
            robot_state = GameState.get_state(robot_state_idx)
            best_action = self.policy(robot_state)
            new_state = robot_state.make_action(best_action[0], best_action[1])

            if robot_state.get_score('child') < new_state.get_score('child'):
                reward = 1
            else:
                reward = -1
            
            example = (robot_state, best_action, reward, new_state)
            examples.append(example)

        return examples

if __name__ == "__main__":
    pass
 