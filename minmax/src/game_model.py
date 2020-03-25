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

# cartesian product of all the dimensions your state is comprised of
# states are all possible combination of ball position that are valid
# 12 states for the first ball of the robot
# 11 states for the second ball of the robot
# 12 states for the first ball of the child
# 11 states for the second ball of the child
# number of states = (12*11)^2*2
# actions = 10
# gamma = 0.99 #because we have a deterministic environment
# reward = scores


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
