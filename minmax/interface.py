#!/usr/bin/env python
import pygame as pg
import pygame.locals
import numpy as np
from minmax.game_model import GameState
from minmax.robot_decision import Robot

class View(object):
    def __init__(self):
        pg.init()
        self.surface = pg.display.set_mode((1920, 1080))
        pg.display.set_caption('Minicomputer Tug of War')

    def draw_empty_board(self):
        self.surface.fill((255, 255, 255))
        rectangles = {"min_score": [80, 270, 180, 180], "max_score": [1620, 270, 180, 180], "800": [360, 180, 180, 180],
                      "400": [540, 180, 180, 180], "200": [360, 360, 180, 180], "100": [540, 360, 180, 180],
                      "80": [780, 180, 180, 180], "40": [960, 180, 180, 180], "20": [780, 360, 180, 180],
                      "10": [960, 360, 180, 180], "8": [1200, 180, 180, 180], "4": [1380, 180, 180, 180],
                      "2": [1200, 360, 180, 180], "1": [1380, 360, 180, 180]}

        for rect in rectangles:
            rect_pos = rectangles[rect]
            color = (0, 0, 0)
            pg.draw.rect(self.surface, color, pg.Rect(rect_pos), 2)

    def draw_score(self, player, value):
        # TODO: Display the score of 'value' for either the child or the robot
        rectangles = {"min_score": [80, 270, 180, 180], "max_score": [1620, 270, 180, 180], "800": [360, 180, 180, 180],
                      "400": [540, 180, 180, 180], "200": [360, 360, 180, 180], "100": [540, 360, 180, 180],
                      "80": [780, 180, 180, 180], "40": [960, 180, 180, 180], "20": [780, 360, 180, 180],
                      "10": [960, 360, 180, 180], "8": [1200, 180, 180, 180], "4": [1380, 180, 180, 180],
                      "2": [1200, 360, 180, 180], "1": [1380, 360, 180, 180]}
        font = pg.font.SysFont("Roboto", 60)

        text = font.render(str(value), True, (0, 0, 0))
        if player == 'child':
            rectangle_position = rectangles["max_score"]
        else:
            rectangle_position = rectangles["min_score"]
        position_x = rectangle_position[0] + int(0.15*rectangle_position[2])
        position_y = rectangle_position[1] + int(0.35*rectangle_position[3])

        self.surface.blit(text, (position_x, position_y))

    def draw_balls(self, balls):
        rectangles = {"min_score": [80, 270, 180, 180], "max_score": [1620, 270, 180, 180], "800": [360, 180, 180, 180],
                      "400": [540, 180, 180, 180], "200": [360, 360, 180, 180], "100": [540, 360, 180, 180],
                      "80": [780, 180, 180, 180], "40": [960, 180, 180, 180], "20": [780, 360, 180, 180],
                      "10": [960, 360, 180, 180], "8": [1200, 180, 180, 180], "4": [1380, 180, 180, 180],
                      "2": [1200, 360, 180, 180], "1": [1380, 360, 180, 180]}

        rectangle_lookup = [["800", "400", "80", "40", "8", "4"],
                            ["200", "100", "20", "10", "2", "1"]]

        robot_ball_one = balls['robot'][0]
        robot_ball_two = balls['robot'][1]
        child_ball_one = balls['child'][0]
        child_ball_two = balls['child'][1]

        robot_ball_one_idx = np.ravel_multi_index(robot_ball_one, [2, 6])
        robot_ball_two_idx = np.ravel_multi_index(robot_ball_two, [2, 6])
        child_ball_one_idx = np.ravel_multi_index(child_ball_one, [2, 6])
        child_ball_two_idx = np.ravel_multi_index(child_ball_two, [2, 6])

        for square_idx in range(2*6):
            is_robot_ball_one = False
            is_robot_ball_two = False
            is_child_ball_one = False
            is_child_ball_two = False
            font = pg.font.SysFont("Roboto", 40)

            if (robot_ball_one_idx == square_idx):
                is_robot_ball_one = True
            if (robot_ball_two_idx == square_idx):
                is_robot_ball_two = True

            if (child_ball_one_idx == square_idx):
                is_child_ball_one = True
            if (child_ball_two_idx == square_idx):
                is_child_ball_two = True

            is_robot_ball = is_robot_ball_one or is_robot_ball_two
            is_child_ball = is_child_ball_one or is_child_ball_two

            if is_robot_ball and is_child_ball:
                robot_color = (83, 109, 254)
                child_color = (211, 47, 47)
                radius = 30

                square_position = np.unravel_index(square_idx, [2, 6])
                # pixel position
                rect = rectangles[rectangle_lookup[square_position[0]]
                                  [square_position[1]]]

                robot_position_x = rect[0] + int(0.30*rect[2])
                robot_position_y = rect[1] + int(0.30*rect[3])
                child_position_x = rect[0] + int(0.70*rect[2])
                child_position_y = rect[1] + int(0.70*rect[3])

                pg.draw.circle(self.surface, robot_color,
                               (robot_position_x, robot_position_y), radius)
                if is_robot_ball_one:
                    text = font.render('A', True, (255, 255, 255))
                    self.surface.blit(
                        text, (robot_position_x-10, robot_position_y-10))
                elif is_robot_ball_two:
                    text = font.render('B', True, (255, 255, 255))
                    self.surface.blit(
                        text, (robot_position_x-10, robot_position_y-10))

                pg.draw.circle(self.surface, child_color,
                               (child_position_x, child_position_y), radius)
                if is_child_ball_one:
                    text = font.render('A', True, (255, 255, 255))
                    self.surface.blit(
                        text, (child_position_x-10, child_position_y-10))
                elif is_child_ball_two:
                    text = font.render('B', True, (255, 255, 255))
                    self.surface.blit(
                        text, (child_position_x-10, child_position_y-10))

            elif is_robot_ball:
                robot_color = (83, 109, 254)
                radius = 30

                square_position = np.unravel_index(square_idx, [2, 6])
                rect = rectangles[rectangle_lookup[square_position[0]]
                                  [square_position[1]]]

                position_x = rect[0] + int(0.5*rect[2])
                position_y = rect[1] + int(0.5*rect[3])

                pg.draw.circle(self.surface, robot_color,
                               (position_x, position_y), radius)
                if is_robot_ball_one:
                    text = font.render('A', True, (255, 255, 255))
                    self.surface.blit(text, (position_x-10, position_y-10))
                elif is_robot_ball_two:
                    text = font.render('B', True, (255, 255, 255))
                    self.surface.blit(text, (position_x-10, position_y-10))

            elif is_child_ball:
                child_color = (211, 47, 47)
                radius = 30

                square_position = np.unravel_index(square_idx, [2, 6])
                rect = rectangles[rectangle_lookup[square_position[0]]
                                  [square_position[1]]]
                position_x = rect[0] + int(0.5*rect[2])
                position_y = rect[1] + int(0.5*rect[3])

                pg.draw.circle(self.surface, child_color,
                               (position_x, position_y), radius)
                if is_child_ball_one:
                    text = font.render('A', True, (255, 255, 255))
                    self.surface.blit(text, (position_x-10, position_y-10))
                elif is_child_ball_two:
                    text = font.render('B', True, (255, 255, 255))
                    self.surface.blit(text, (position_x-10, position_y-10))

    def draw_outcome(self, is_child_turn):
        font = pg.font.SysFont("Roboto", 85)
        position_x = 800
        position_y = 900
        game_state = GameState()
        is_finished =  game_state.isFinished()
        print(is_finished)

        ### THIS IS NOT DISPLAYING THE OUTPUT
        if not is_child_turn and is_finished:
            text = font.render('Child Wins', True, (0, 0, 0))
            self.surface.blit(text, (position_x, position_y))
        elif is_child_turn and is_finished:
            text = font.render('Robot Wins', True, (0, 0, 0))
            self.surface.blit(text, (position_x, position_y))
    
    def draw_explanation(self, is_child_turn, game_state):
        font = pg.font.SysFont("Roboto", 45)
        position_x = 50
        position_y = 900
        robot = Robot()
        action = robot.policy(game_state)
        game_state, current_score, scores, action, other_actions, rewards = robot.give_text(action,game_state)

        action_dict = {
              0: 'moving the ball A diagonally down left', 
              1: 'moving the ball A down',
              2: 'moving the ball A diagonally down right',
              3: 'moving the ball A left',
              4: 'not moving ball A',
              5: 'moving the ball A right',
              6: 'moving the ball A diagonally up left',
              7: 'moving the ball A up',
              8: 'moving the ball A diagonally up right',
              9: 'moving the ball B diagonally down left',
              10: 'moving the ball B down',
              11: 'moving the ball B down right',
              12: 'moving the ball B left',
              13: 'not moving ball B',
              14: 'moving the ball B right',
              15: 'moving the ball B diagonally up left',
              16: 'moving the ball B up',
              17: 'moving the ball B diagonally up right'
              }
        
        if current_score > scores[0]:
            exp = f"By {action_dict[action]} I get {current_score} points, while by {action_dict[other_actions[0]]} I would get points {abs(rewards[0])} less"
        elif current_score < scores[0]:
            exp = f"By {action_dict[action]} I get {current_score} points, while by {action_dict[other_actions[0]]} I would get points {abs(rewards[0])} more"
        else:
            exp = f"By {action_dict[action]} I get {current_score} points, and by {action_dict[other_actions[0]]} I would get the same amount of points"

        if is_child_turn and current_score!=1000:
            text = font.render(exp, True, (0, 0, 0))
            self.surface.blit(text, (position_x, position_y))

    def update(self, game_state):
        self.draw_empty_board()
        self.draw_score('robot', game_state.get_score('robot'))
        self.draw_score('child', game_state.get_score('child'))
        self.draw_outcome(game_state.is_child_turn)
        self.draw_explanation(game_state.is_child_turn, game_state)
        self.draw_balls(game_state.balls)
        pg.display.update()


if __name__ == "__main__":
    from minmax import GameState
    import time
    state = GameState()
    visualization = View()

    while not state.isFinished():
        visualization.update(state)
        time.sleep(0.25)

        valid_actions = state.valid_actions()
        valid_action_idx = np.random.randint(len(valid_actions))
        action = valid_actions[valid_action_idx]
        state, reward, done, info = state.make_action(action)

    visualization.update(state)
    time.sleep(1.5)
