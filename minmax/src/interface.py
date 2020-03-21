#!/usr/bin/env python
import pygame as pg
import pygame.locals
import numpy as np

# draw some text into an area of a surface
# automatically wraps words
# returns any text that didn't get blitted

# You can use the previous values as helpers for the position
# board_pos_dic = {"min_score":[80, 270, 180, 180], "max_score":[1620, 270, 180, 180], "800":[360, 180, 180, 180],
#                  "400":[540, 180, 180, 180], "200":[360, 360, 180, 180], "100":[540, 360, 180, 180],
#                  "80":[780, 180, 180, 180], "40": [960, 180, 180, 180], "20":[780, 360, 180, 180],
#                  "10":[960, 360, 180, 180], "8":[1200, 180, 180, 180], "4":[1380, 180, 180, 180],
#                  "2":[1200, 360, 180, 180], "1":[1380, 360, 180, 180]}

# board_color_dic = {"min_score":[255, 255, 255], "max_score":[255, 255, 255], "lightbrown":[215, 204, 200],
#                   "lightpink":[193, 159, 254], "lightred":[255, 205, 210], "white":[255, 255, 255]}


class View(object):
    def __init__(self):
        # TODO: Initialize a pygame window
        return

    def draw_empty_board(self):
        # TODO: Draw the constant parts of the visualization, i.e., all the 
        # things that do not move/change in the game, like the squares or
        # or the title and alike
        return

    def draw_score(self, player, value):
        # TODO: Display the score of 'value' for either the child or the robot
        return

    def draw_balls(self, balls):
        # TODO: Draw the balls in the respective square. Note that there may
        # be two balls in a single square
        robot_ball_one = game_state.balls['robot'][0]
        robot_ball_two = game_state.balls['robot'][1]
        child_ball_two = game_state.balls['child'][0]
        child_ball_two = game_state.balls['child'][1]

        robot_ball_one_idx = np.unravel_index(robot_ball_one, [2, 6])
        robot_ball_two_idx = np.unravel_index(robot_ball_two, [2, 6])
        child_ball_two_idx = np.unravel_index(child_ball_one, [2, 6])
        child_ball_two_idx = np.unravel_index(child_ball_two, [2, 6])

        for square_idx in range(2*6):
            is_robot_ball = False
            is_child_ball = False

            if (robot_ball_one_idx == square_idx
                    or robot_ball_two_idx == square_idx):
                is_robot_ball = True

            if (child_ball_one_idx == square_idx
                    or child_ball_two_idx == square_idx):
                is_child_ball = True

            # TODO: draw the balls for the respective field

        return

    def update(self, game_state):
        self.draw_empty_board()
        self.draw_score('robot', game_state.get_score('robot'))
        self.draw_score('child', game_state.get_score('child'))
        self.draw_balls(game_state.balls)

        # TODO: update the pygame visualizazion
        return


if __name__ == "__main__":
    state = GameState()
    game = View()

    while not state.isFinished():
        game.update(state)

        valid_actions = state.valid_actions()
        valid_action_idx = np.random.randint(len(valid_actions))
        action, ball_id = valid_actions[valid_action_idx]
        state = state.make_action(action, ball_id)

    game.update(state)
