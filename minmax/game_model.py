import numpy as np
from copy import deepcopy


class GameState(object):
    def __init__(self):
        self.balls = {
            'robot': [np.array([0, 0]), np.array([1, 0])],
            'child': [np.array([0, 5]), np.array([1, 5])]
            }
        self.is_child_turn = True

    def get_score(self, player):
        values = np.array([[800, 400, 80, 40, 8, 4],
                           [200, 100, 20, 10, 2, 1]])
        balls = self.balls[player]

        total_score = 0
        for ball_position in balls:
            score = values[ball_position[0], ball_position[1]]
            total_score += score

        return total_score

    def isValid(self):
        balls = self.balls['robot'] + self.balls['child']
        GAME_MAX_CELL_X = 5
        GAME_MAX_CELL_Y = 1

        for ball_position in balls:
            if ball_position[1] > GAME_MAX_CELL_X:
                return False
            elif ball_position[0] > GAME_MAX_CELL_Y:
                return False
            elif ball_position[0] < 0:
                return False
            elif ball_position[1] < 0:
                return False

        robot_ball1 = self.balls['robot'][0]
        robot_ball2 = self.balls['robot'][1]

        if (robot_ball1[0] == robot_ball2[0]
                and robot_ball1[1] == robot_ball2[1]):
            return False

        child_ball1 = self.balls['child'][0]
        child_ball2 = self.balls['child'][1]

        if (child_ball1[0] == child_ball2[0]
                and child_ball1[1] == child_ball2[1]):
            return False

        return True

    def make_action(self, action, ball_id):
        if self.is_child_turn:
            player = 'child'
        else:
            player = 'robot'
        direction = {
                        'up': np.array([-1, 0]),
                        'down': np.array([1, 0]),
                        'left': np.array([0, -1]),
                        'right': np.array([0, 1]),
                        'up_left': np.array([-1, -1]),
                        'up_right': np.array([-1, 1]),
                        'down_left': np.array([1, -1]),
                        'down_right': np.array([1, 1])
                    }

        ball_position = self.balls[player][ball_id]
        new_position = ball_position + direction[action]

        # deepcopy for having two copies of the same state and later modify it
        new_state = deepcopy(self)
        new_state.balls[player][ball_id] = new_position
        new_state.is_child_turn = not self.is_child_turn
        return new_state

    def valid_actions(self):
        if self.is_child_turn:
            player = 'child'
        else:
            player = 'robot'

        if player == 'child':
            actions = ['up', 'down', 'left', 'up_left', 'down_left']
        else:
            actions = ['up', 'down', 'right', 'up_right', 'down_right']

        valid_actions = list()
        for ball_id in [0, 1]:
            for action in actions:
                new_state = self.make_action(action, ball_id)
                if new_state.isValid():
                    valid_actions.append((action, ball_id))

        return valid_actions

    def isFinished(self):
        robot_score = self.get_score('robot')
        child_score = self.get_score('child')

        if child_score >= robot_score:
            return True
        else:
            return False

    def __repr__(self):
        values = np.zeros([2, 6])
        robot_ball1 = self.balls['robot'][0]
        robot_ball2 = self.balls['robot'][1]
        child_ball1 = self.balls['child'][0]
        child_ball2 = self.balls['child'][1]

        values[robot_ball1[0], robot_ball1[1]] = 1
        values[robot_ball2[0], robot_ball2[1]] = 1

        values[child_ball1[0], child_ball1[1]] = 2
        values[child_ball2[0], child_ball2[1]] = 2

        return str(values)

    @staticmethod
    def get_action_id(action, ball_id):
        action_id = {
                     ('left', 0): 0,
                     ('left', 1): 1,
                     ('right', 0): 2,
                     ('right', 1): 3,
                     ('up', 0): 4,
                     ('up', 1): 5,
                     ('down', 0): 6,
                     ('down', 1): 7,
                     ('up_left', 0): 8,
                     ('up_left', 1): 9,
                     ('down_left', 0): 10,
                     ('down_left', 1): 11,
                     ('up_right', 0): 12,
                     ('up_right', 1): 13,
                     ('down_right', 0): 14,
                     ('down_right', 1): 15
                    }
        return action_id[(action,ball_id)]

    @staticmethod
    def get_state_id(state):
        # convert the state to array index
        robot_ball1 = state.balls['robot'][0]
        robot_ball2 = state.balls['robot'][1]
        child_ball1 = state.balls['child'][0]
        child_ball2 = state.balls['child'][1]
        turn = state.is_child_turn

        robot_ball1_idx = np.ravel_multi_index(robot_ball1, [2,6])
        robot_ball2_idx = np.ravel_multi_index(robot_ball2, [2,6])
        child_ball1_idx = np.ravel_multi_index(child_ball1, [2,6])
        child_ball2_idx = np.ravel_multi_index(child_ball2, [2,6])
        turn_idx = 1 if turn else 0
        
        multi_idx = [
                     robot_ball1_idx,
                     robot_ball2_idx,
                     child_ball1_idx,
                     child_ball2_idx,
                     turn_idx
                    ]
        
        #convert the array index to integer number
        array_shape = (12,12,12,12,2)
        state_idx = np.ravel_multi_index(multi_idx, array_shape)

        return state_idx

    @staticmethod
    def get_state(state_idx):
        # array_range = 12*12*12*12*2
        array_shape = (12,12,12,12,2)
        multi_idx = np.unravel_index(state_idx, array_shape)
        
        (robot_ball1_idx, 
         robot_ball2_idx,
         child_ball1_idx,
         child_ball2_idx,
         turn_idx
        ) = multi_idx
        
        turn = True if turn_idx == 1 else False
        robot_ball1 = np.unravel_index(robot_ball1_idx, [2,6])
        robot_ball2 = np.unravel_index(robot_ball2_idx, [2,6])
        child_ball1 = np.unravel_index(child_ball1_idx, [2,6])
        child_ball2 = np.unravel_index(child_ball2_idx, [2,6])
        
        state = GameState()
        # convert the state to array index
        state.balls['robot'][0] = np.array(robot_ball1)
        state.balls['robot'][1] = np.array(robot_ball2)
        state.balls['child'][0] = np.array(child_ball1)
        state.balls['child'][1] = np.array(child_ball2)
        state.is_child_turn = turn

        return state

    def get_winner(self):
        #TODO: refactor when time
        pass


if __name__ == "__main__":
    state = GameState()
    while not state.isFinished():
        print(state)
        print(state.valid_actions())

        valid_actions = state.valid_actions()
        valid_action_idx = np.random.randint(len(valid_actions))
        action, ball_id = valid_actions[valid_action_idx]
        state = state.make_action(action, ball_id)

    print(state)
