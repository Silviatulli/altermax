#!/usr/bin/env python
import pygame as pg
import pygame.locals
import numpy as np

class View(object):
    def __init__(self):
        pg.init()
        self.surface = pg.display.set_mode((1920, 1080))
        pg.display.set_caption('Minicomputer Tug of War')

    def draw_empty_board(self):
        self.surface.fill((255,255,255))
        rectangles = {"min_score":[80, 270, 180, 180], "max_score":[1620, 270, 180, 180], "800":[360, 180, 180, 180],
                 "400":[540, 180, 180, 180], "200":[360, 360, 180, 180], "100":[540, 360, 180, 180],
                 "80":[780, 180, 180, 180], "40": [960, 180, 180, 180], "20":[780, 360, 180, 180],
                 "10":[960, 360, 180, 180], "8":[1200, 180, 180, 180], "4":[1380, 180, 180, 180],
                 "2":[1200, 360, 180, 180], "1":[1380, 360, 180, 180]}
        
        for rect in rectangles:
            rect_pos = rectangles[rect]
            color = (0, 0, 0)
            pg.draw.rect(self.surface, color, pg.Rect(rect_pos), 2)

    def draw_score(self, player, value):
        # TODO: Display the score of 'value' for either the child or the robot
        rectangles = {"min_score":[80, 270, 180, 180], "max_score":[1620, 270, 180, 180], "800":[360, 180, 180, 180],
            "400":[540, 180, 180, 180], "200":[360, 360, 180, 180], "100":[540, 360, 180, 180],
            "80":[780, 180, 180, 180], "40": [960, 180, 180, 180], "20":[780, 360, 180, 180],
            "10":[960, 360, 180, 180], "8":[1200, 180, 180, 180], "4":[1380, 180, 180, 180],
            "2":[1200, 360, 180, 180], "1":[1380, 360, 180, 180]}
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
        # TODO: Draw the balls in the respective square. Note that there may
        # be two balls in a single square

                
                
        rectangles = {"min_score":[80, 270, 180, 180], "max_score":[1620, 270, 180, 180], "800":[360, 180, 180, 180],
            "400":[540, 180, 180, 180], "200":[360, 360, 180, 180], "100":[540, 360, 180, 180],
            "80":[780, 180, 180, 180], "40": [960, 180, 180, 180], "20":[780, 360, 180, 180],
            "10":[960, 360, 180, 180], "8":[1200, 180, 180, 180], "4":[1380, 180, 180, 180],
            "2":[1200, 360, 180, 180], "1":[1380, 360, 180, 180]}

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
            is_robot_ball = False
            is_child_ball = False
            #font_color = (255, 255, 255)
            #font = pg.font.SysFont("Roboto", 60)

            if (robot_ball_one_idx == square_idx
                    or robot_ball_two_idx == square_idx):
                is_robot_ball = True

            if (child_ball_one_idx == square_idx
                    or child_ball_two_idx == square_idx):
                is_child_ball = True

            if is_robot_ball and is_child_ball:
                robot_color = (83, 109, 254)
                child_color = (211, 47, 47)
                radius = 30

                square_position = np.unravel_index(square_idx, [2, 6])
                rect = rectangles[rectangle_lookup[square_position[0]][square_position[1]]]

                robot_position_x = rect[0] + int(0.30*rect[2])
                robot_position_y = rect[1] + int(0.30*rect[3])
                child_position_x = rect[0] + int(0.70*rect[2])
                child_position_y = rect[1] + int(0.70*rect[3])

                pg.draw.circle(self.surface, robot_color, (robot_position_x, robot_position_y), radius)
                pg.draw.circle(self.surface, child_color, (child_position_x, child_position_y), radius)

            elif is_robot_ball:
                robot_color = (83, 109, 254)
                radius = 30

                square_position = np.unravel_index(square_idx, [2, 6])
                rect = rectangles[rectangle_lookup[square_position[0]][square_position[1]]]
                position_x = rect[0] + int(0.5*rect[2])
                position_y = rect[1] + int(0.5*rect[3])
                
                pg.draw.circle(self.surface, robot_color, (position_x, position_y), radius)
            
            elif is_child_ball:
                child_color = (211, 47, 47)
                radius = 30

                square_position = np.unravel_index(square_idx, [2, 6])
                rect = rectangles[rectangle_lookup[square_position[0]][square_position[1]]]
                position_x = rect[0] + int(0.5*rect[2])
                position_y = rect[1] + int(0.5*rect[3])
                
                pg.draw.circle(self.surface, child_color, (position_x, position_y), radius)
            
           


    def update(self, game_state):
        self.draw_empty_board()
        self.draw_score('robot', game_state.get_score('robot'))
        self.draw_score('child', game_state.get_score('child'))
        self.draw_balls(game_state.balls)

        # TODO: update the pygame visualizazion
        # # If the game has ended and the child wins:
        # if self.finished_game and self.child_win:

        #     self.child_win_board.draw(self.background)

        #     #self.play_again_board.draw(self.background)

        # elif self.finished_game and not self.child_win:

        #     self.robot_win_board.draw(self.background)

        pg.display.update()
        return


if __name__ == "__main__":
    from game_model import GameState
    import time
    state = GameState()
    game = View()

    while not state.isFinished():
        game.update(state)
        time.sleep(0.5)

        valid_actions = state.valid_actions()
        valid_action_idx = np.random.randint(len(valid_actions))
        action, ball_id = valid_actions[valid_action_idx]
        state = state.make_action(action, ball_id)

    game.update(state)