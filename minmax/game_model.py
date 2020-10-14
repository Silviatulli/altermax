import numpy as np
from enum import IntEnum


class Action(IntEnum):
    ball1_down_left = 0
    ball1_down = 1
    ball1_down_right = 2
    ball1_left = 3
    ball1_none = 4
    ball1_right = 5
    ball1_up_left = 6
    ball1_up = 7
    ball1_up_right = 8
    ball2_down_left = 9
    ball2_down = 10
    ball2_down_right = 11
    ball2_left = 12
    ball2_none = 13
    ball2_right = 14
    ball2_up_left = 15
    ball2_up = 16
    ball2_up_right = 17


CARDINAL_DIRECTIONS = np.stack(
    np.unravel_index(np.arange(9), (3, 3)), axis=1) - 1
DIRECTION = np.vstack((CARDINAL_DIRECTIONS, CARDINAL_DIRECTIONS))
OCCUPIED = np.pad(np.zeros((2, 6)), 1, constant_values=1)


class GameState(object):
    __slots__ = ['balls', 'is_child_turn']

    def __init__(self):
        self.balls = {
            'robot': [(0, 0), (1, 0)],
            'child': [(0, 5), (1, 5)]
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

    def make_action(self, action):
        if self.is_child_turn:
            player = 'child'
        else:
            player = 'robot'

        ball_id = 0 if action < 9 else 1
        ball_position = self.balls[player][ball_id]
        y = ball_position[0] + DIRECTION[action][0]
        x = ball_position[1] + DIRECTION[action][1]
        new_position = (y, x)
        self.balls[player][ball_id] = new_position
        self.is_child_turn = not self.is_child_turn

        if not self.isValid():
            raise Exception()
        return self, 0, self.isFinished(), None

    def valid_actions(self):
        if self.is_child_turn:
            player = 'child'
        else:
            player = 'robot'

        if player == 'child':
            actions = [
                Action.ball1_down_left,
                Action.ball1_down,
                Action.ball1_left,
                Action.ball1_up_left,
                Action.ball1_up,
                Action.ball2_down_left,
                Action.ball2_down,
                Action.ball2_left,
                Action.ball2_up_left,
                Action.ball2_up
            ]
        else:
            actions = [
                Action.ball1_down,
                Action.ball1_down_right,
                Action.ball1_right,
                Action.ball1_up,
                Action.ball1_up_right,
                Action.ball2_down,
                Action.ball2_down_right,
                Action.ball2_right,
                Action.ball2_up,
                Action.ball2_up_right
            ]

        valid_actions = list()

        for action in actions:
            ball_id = 0 if action < 9 else 1
            other_ball_id = (ball_id + 1) % 2
            pos = self.balls[player][ball_id]
            y2 = self.balls[player][other_ball_id][0] + 1
            x2 = self.balls[player][other_ball_id][1] + 1
            y = pos[0] + DIRECTION[action][0] + 1
            x = pos[1] + DIRECTION[action][1] + 1
            if OCCUPIED[y, x]:
                continue
            elif y == y2 and x == x2:
                continue
            else:
                valid_actions.append(action)

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
    def get_state_id(state):
        # convert the state to array index
        robot_ball1 = state.balls['robot'][0]
        robot_ball2 = state.balls['robot'][1]
        child_ball1 = state.balls['child'][0]
        child_ball2 = state.balls['child'][1]
        turn = state.is_child_turn

        robot_ball1_idx = np.ravel_multi_index(robot_ball1, [2, 6])
        robot_ball2_idx = np.ravel_multi_index(robot_ball2, [2, 6])
        child_ball1_idx = np.ravel_multi_index(child_ball1, [2, 6])
        child_ball2_idx = np.ravel_multi_index(child_ball2, [2, 6])
        turn_idx = 1 if turn else 0

        multi_idx = [
            robot_ball1_idx,
            robot_ball2_idx,
            child_ball1_idx,
            child_ball2_idx,
            turn_idx
        ]

        # convert the array index to integer number
        array_shape = (12, 12, 12, 12, 2)
        state_idx = np.ravel_multi_index(multi_idx, array_shape)

        return state_idx

    @staticmethod
    def get_state(state_idx):
        # array_range = 12*12*12*12*2
        array_shape = (12, 12, 12, 12, 2)
        multi_idx = np.unravel_index(state_idx, array_shape)

        (robot_ball1_idx,
         robot_ball2_idx,
         child_ball1_idx,
         child_ball2_idx,
         turn_idx
         ) = multi_idx

        turn = True if turn_idx == 1 else False
        robot_ball1 = np.unravel_index(robot_ball1_idx, [2, 6])
        robot_ball2 = np.unravel_index(robot_ball2_idx, [2, 6])
        child_ball1 = np.unravel_index(child_ball1_idx, [2, 6])
        child_ball2 = np.unravel_index(child_ball2_idx, [2, 6])

        state = GameState()
        # convert the state to array index
        state.balls['robot'][0] = tuple(robot_ball1)
        state.balls['robot'][1] = tuple(robot_ball2)
        state.balls['child'][0] = tuple(child_ball1)
        state.balls['child'][1] = tuple(child_ball2)
        state.is_child_turn = turn

        return state

    @property
    def winner(self):
        if not self.isFinished():
            return None

        if self.is_child_turn:
            return 'robot'
        else:
            return 'child'


if __name__ == "__main__":
    import tqdm
    for _ in tqdm.tqdm(range(2000)):
        state = GameState()
        done = False
        while not done:
            valid_actions = state.valid_actions()
            valid_action_idx = np.random.randint(len(valid_actions))
            action = valid_actions[valid_action_idx]
            state, reward, done, info = state.make_action(action)
