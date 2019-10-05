#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import sys
from src.utils import *
from minmax.srv import *
import pygame as pg
from pygame.locals import *
import math

NODE_NAME = 'minmax_game_interface'

"""
Module responsible for the Game Interface


"""

# draw some text into an area of a surface
# automatically wraps words
# returns any text that didn't get blitted


board_pos_dic = {"min_score":[80, 270, 180, 180], "max_score":[1620, 270, 180, 180], "800":[360, 180, 180, 180],
                 "400":[540, 180, 180, 180], "200":[360, 360, 180, 180], "100":[540, 360, 180, 180],
                 "80":[780, 180, 180, 180], "40": [960, 180, 180, 180], "20":[780, 360, 180, 180],
                 "10":[960, 360, 180, 180], "8":[1200, 180, 180, 180], "4":[1380, 180, 180, 180],
                 "2":[1200, 360, 180, 180], "1":[1380, 360, 180, 180]}

board_color_dic = {"min_score":[255, 255, 255], "max_score":[255, 255, 255], "lightbrown":[215, 204, 200],
                  "lightpink":[193, 159, 254], "lightred":[255, 205, 210], "white":[255, 255, 255]}


# define the RGB value for white,
#  green, blue colour .
white = [255, 255, 255]
green = [0, 0, 255]
blue = [0, 255, 0]
black = [0, 0, 0]
lightpink = [193, 159, 254]
lightred = [255, 205, 210]
lightbrown = [215, 204, 200]



class BoardSquare:
    def __init__(self, rect_pos, text, score, color):
        self.score = score
        self.rect_pos = rect_pos
        self.width = 3
        self.boundary_pos = [rect_pos[0] - self.width, rect_pos[1] - self.width,
                             rect_pos[2] + 2*self.width, rect_pos[3] + 2*self.width]
        self.text_str = text
        self.rect = pg.Rect(rect_pos)
        #self.font_color = (255, 255, 255)
        self.font = pg.font.SysFont("Roboto", 72)
        self.color = color


    def update_text(self, new_text):
        self.text_str = new_text


    def draw(self, surface):
        pg.draw.rect(surface, black, pg.Rect(self.boundary_pos), 0)
        pg.draw.rect(surface, self.color, pg.Rect(self.rect_pos), 0)
        rect_center_x = self.rect_pos[0] + int(0.15*self.rect_pos[2])
        rect_center_y = self.rect_pos[1] + int(0.35*self.rect_pos[3])
        surface.blit(self.font.render(self.text_str, True, (0, 0, 0)), (rect_center_x, rect_center_y))
        return

    def get_center_pos(self):
        return [self.rect_pos[0] + int(0.5*self.rect_pos[2]), self.rect_pos[1] + int(0.5*self.rect_pos[3])]

    def get_score(self):
        return self.score



class CounterBoardSquare:
    def __init__(self, rect_pos, text, score, color):
        self.score = score
        self.width = 3
        self.text_size = 60
        self.rect_pos = rect_pos
        self.boundary_pos = [rect_pos[0] - self.width, rect_pos[1] - self.width,
                             rect_pos[2] + 2 * self.width, rect_pos[3] + 2 * self.width]
        self.text_str = text
        self.rect = pg.Rect(rect_pos)
        #self.font_color = (255, 255, 255)
        self.font = pg.font.SysFont("Roboto", self.text_size)
        self.color = color


    def update_text(self, new_text):
        self.text_str = new_text


    def draw(self, surface):
        pg.draw.rect(surface, black, pg.Rect(self.boundary_pos), 0)
        pg.draw.rect(surface, self.color, pg.Rect(self.rect_pos), 0)
        rect_center_x = self.rect_pos[0] + int(0.15*self.rect_pos[2])
        rect_center_y = self.rect_pos[1] + int(0.35*self.rect_pos[3])
        surface.blit(self.font.render(self.text_str, True, (0, 0, 0)), (rect_center_x, rect_center_y))
        return

    def get_center_pos(self):
        return [self.rect_pos[0] + int(0.5*self.rect_pos[2]), self.rect_pos[1] + int(0.5*self.rect_pos[3])]

    def get_score(self):
        return self.score

    def set_score(self, score):
        self.text_str = str(score)
        self.score = score



class CircleBall:
    def __init__(self, pos, id, player_id, score):
        self.pos = pos
        self.width = 3
        self.previous_pos = None
        self.id = id
        self.text_str = str(self.id)
        self.text_size = 60
        self.radius = 30
        self.bounding_radius = self.radius + self.width
        self.score = score
        self.circle_bb = pg.rect.Rect(self.pos[0]-int(0.8*self.radius), self.pos[1]-int(0.8*self.radius), int(2.5*self.radius), int(2.5*self.radius))
        self.dragging = False

        self.player_id = player_id

        if self.player_id == player_type_dic['max']:
            self.color = (211, 47, 47)
        else:
            self.color = (83, 109, 254)

        self.font_color = (255, 255, 255)
        self.font = pg.font.SysFont("Roboto", self.text_size)



    def move_ball(self, pos):
        self.pos = pos
        # Update bounding box coordinates
        self.circle_bb = pg.rect.Rect(self.pos[0] - int(0.5 * self.radius), self.pos[1] - int(0.5 * self.radius),
                                      2 * self.radius, 2 * self.radius)

    def move_ball_to_score(self, score, board_list):
        best_board = None

        for board in board_list:

           if board.score == score:
               best_board = board

        best_board_score = best_board.get_score()


        # IF the score is the same, then it was a child mistake
        if self.score == best_board_score or self.score < best_board_score:
            return False

        # Update variables
        self.score = best_board_score


        # Now we are certain we can move the ball to the pos
        self.move_ball(pos=best_board.get_center_pos())
        return True


    def draw(self, surface):
        pg.draw.circle(surface, white, (self.pos[0], self.pos[1]), self.bounding_radius)
        pg.draw.circle(surface, self.color, (self.pos[0], self.pos[1]), self.radius)
        surface.blit(self.font.render(self.text_str, True, (255, 255, 255)), (self.pos[0]-int(self.radius*0.5), self.pos[1]-int(self.radius*0.75)))
        return

    def is_colliding(self, mouse_pos):

        if mouse_pos[0] > self.pos[0]-int(self.radius*0.5) and \
                mouse_pos[0] <= self.pos[0] + self.radius and mouse_pos[1] > self.pos[1] - int(self.radius*0.5)\
                and mouse_pos[1] <= self.pos[1] + self.radius:
            return True
        else:
            return False

    def update_to_new_board(self, board_list, min_ball_list, max_ball_list):

        best_board = None
        best_board_distance = float("inf")
        best_board_score = None

        for board in board_list:

            board_center_pos = board.get_center_pos()
            distance = math.sqrt(math.pow(self.pos[0] - board_center_pos[0], 2) + (math.pow(self.pos[1] - board_center_pos[1], 2)))

            if distance < best_board_distance:
                best_board = board
                best_board_score = board.get_score()
                best_board_distance = distance


        # IF the score is the same, then it was a child mistake
        if self.score == best_board_score or self.score > best_board_score:
            return False

        # Update variables
        self.score = best_board_score


        # Now check if we can move the ball there
        for min_ball in min_ball_list:

            #If we have two balls in the same square, we need to move it a little bit to the down-right
            if min_ball.score == self.score:
                new_pos = [best_board.get_center_pos()[0] + int(1.5*self.radius),
                           best_board.get_center_pos()[1] + int(1.5*self.radius)]
                self.move_ball(pos=new_pos)
                return True

        for max_ball in max_ball_list:
            if max_ball.id != self.id:
                if max_ball.score == self.score:
                    # We cannot move to this position
                    return False

        # Now we are certain we can move the ball to the pos
        self.move_ball(pos = best_board.get_center_pos())

        return True


    def get_id(self):
        return self.id

    def get_player_id(self):
        return self.player_id

    def get_score(self):
        return self.score




class GameInterface:
    def __init__(self):

        # Variables
        pg.init()
        self.ttt = pg.display.set_mode ((1920, 1080))
            #, flags=pg.NOFRAME)
        pg.display.set_caption ('Guerra da Mini Calculadora')

        # Game control variables
        self.wait = False
        self.finished_game = False
        self.child_win = False
        self.robot_turn = False
        self.child_turn = True
        self.minimizer_ball_0 = None
        self.minimizer_ball_1 = None
        self.maximizer_ball_0 = None
        self.maximizer_ball_1 = None
        self.total_max_score = None
        self.total_min_score = None
        self.background = None



        # board variables
        self.board = self.initBoard (self.ttt)

        self.min_score_board = CounterBoardSquare(board_pos_dic["min_score"], "0", -1, board_color_dic["min_score"])
        self.max_score_board = CounterBoardSquare(board_pos_dic["max_score"], "0", -1, board_color_dic["max_score"])

        self.box_800_board = BoardSquare(board_pos_dic["800"], " ", 800, lightbrown)
        self.box_400_board = BoardSquare(board_pos_dic["400"], " ", 400, lightpink)
        self.box_200_board = BoardSquare(board_pos_dic["200"], " ", 200, lightred)
        self.box_100_board = BoardSquare(board_pos_dic["100"], " ", 100, white)

        self.box_80_board = BoardSquare(board_pos_dic["80"], " ", 80, lightbrown)
        self.box_40_board = BoardSquare(board_pos_dic["40"], " ", 40, lightpink)
        self.box_20_board = BoardSquare(board_pos_dic["20"], " ", 20, lightred)
        self.box_10_board = BoardSquare(board_pos_dic["10"], " ", 10, white)

        self.box_8_board = BoardSquare(board_pos_dic["8"], " ", 8, lightbrown)
        self.box_4_board = BoardSquare(board_pos_dic["4"], " ", 4, lightpink)
        self.box_2_board = BoardSquare(board_pos_dic["2"], " ", 2, lightred)
        self.box_1_board = BoardSquare(board_pos_dic["1"], " ", 1, white)
        self.child_win_board = BoardSquare([640, 720, 500, 270], "Tu Ganhaste", -1, (211, 47, 47))
        self.robot_win_board = BoardSquare([640, 720, 500, 270], "NAO Ganhou", -1, (83, 109, 254))
        #self.play_again_board = BoardSquare([400, 300, 400, 450], "Joga De Novo", 0, white)

        self.board_list = [self.box_800_board, self.box_400_board, self.box_200_board, self.box_100_board,
                           self.box_80_board, self.box_40_board, self.box_20_board, self.box_10_board,
                           self.box_8_board, self.box_4_board, self.box_2_board, self.box_1_board]

        self.circle_min_ball_1 = CircleBall(self.box_800_board.get_center_pos(), 1, player_type_dic["min"], score=800)
        self.circle_min_ball_0 = CircleBall(self.box_200_board.get_center_pos(), 0, player_type_dic["min"], score=200)
        self.circle_max_ball_0 = CircleBall(self.box_1_board.get_center_pos(), 0, player_type_dic["max"], score=1)
        self.circle_max_ball_1 = CircleBall(self.box_4_board.get_center_pos(), 1, player_type_dic["max"], score=4)

        self.max_balls_list = [self.circle_max_ball_0, self.circle_max_ball_1]
        self.min_balls_list = [self.circle_min_ball_0, self.circle_min_ball_1]

        self.offset_x = None
        self.offset_y = None

        # Services to connect to the Game Manager
        self.manager_to_game_service = rospy.Service('manager_to_game_service', GameState, self.handle_information_from_manager)

        rospy.wait_for_service('game_to_manager_service')
        try:
            self.game_to_manager_proxy = rospy.ServiceProxy('game_to_manager_service', GameState)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e



    def play_game(self):
        return

    def handle_information_from_manager(self, req):

        rospy.loginfo("[Game Interface] Received information from manager.")

        if not self.finished_game:
            # Read message from the manager
            self.finished_game = req.finished_game
            self.wait = req.wait
            self.robot_turn = req.robot_turn
            self.child_turn = req.child_turn
            self.minimizer_ball_0 = req.minimizer_ball_0
            self.minimizer_ball_1 = req.minimizer_ball_1
            self.maximizer_ball_0 = req.maximizer_ball_0
            self.maximizer_ball_1 = req.maximizer_ball_1
            self.total_min_score = self.minimizer_ball_0 + self.minimizer_ball_1
            self.total_max_score = self.maximizer_ball_0 + self.maximizer_ball_1

            # Send message to interface to change the game
            rospy.loginfo("[Game Interface] Finished Game = " + str(self.finished_game))
            rospy.loginfo("[Game Interface] Wait = " + str(self.wait))
            rospy.loginfo("[Game Interface] Robot Turn = " + str(self.robot_turn))
            rospy.loginfo("[Game Interface] Child Turn = " + str(self.child_turn))
            rospy.loginfo("[Game Interface] Max Ball 0 = " + str(self.maximizer_ball_0))
            rospy.loginfo("[Game Interface] Max Ball 1 = " + str(self.maximizer_ball_1))
            rospy.loginfo("[Game Interface] Min Ball 0 = " + str(self.minimizer_ball_0))
            rospy.loginfo("[Game Interface] Min Ball 1 = " + str(self.minimizer_ball_1))

            self.change_game()

        # Respond to Game Manager
        gstate_resp = GameStateResponse()
        gstate_resp.success = True
        return gstate_resp


    def send_information_to_manager(self):

        # Before sending information lets just check if the game ended
        self.check_for_game_ended()

        game_request = GameStateRequest()
        game_request.finished_game = self.finished_game
        game_request.wait = self.wait
        game_request.robot_turn = self.robot_turn
        game_request.child_turn = self.child_turn
        game_request.maximizer_ball_0 = self.circle_max_ball_0.get_score()
        game_request.maximizer_ball_1 = self.circle_max_ball_1.get_score()
        game_request.minimizer_ball_0 = self.circle_min_ball_0.get_score()
        game_request.minimizer_ball_1 = self.circle_min_ball_1.get_score()

        rospy.loginfo("[Game Interface] Sending information to Manager")
        rospy.loginfo("[Game Interface] Finished Game = " + str(self.finished_game))
        rospy.loginfo("[Game Interface] Wait = " + str(self.wait))
        rospy.loginfo("[Game Interface] Robot Turn = " + str(self.robot_turn))
        rospy.loginfo("[Game Interface] Child Turn = " + str(self.child_turn))
        rospy.loginfo("[Game Interface] Max Ball 0 = " + str(game_request.maximizer_ball_0))
        rospy.loginfo("[Game Interface] Max Ball 1 = " + str(game_request.maximizer_ball_1))
        rospy.loginfo("[Game Interface] Min Ball 0 = " + str(game_request.minimizer_ball_0))
        rospy.loginfo("[Game Interface] Min Ball 1 = " + str(game_request.minimizer_ball_1))

        # Call the manager to game service and wait for response
        game_response = self.game_to_manager_proxy.call(game_request)
        return game_response

    def change_game(self):

        # Move the balls to the correct place
        self.circle_min_ball_0.move_ball_to_score(score=self.minimizer_ball_0, board_list=self.board_list)
        self.circle_min_ball_1.move_ball_to_score(score=self.minimizer_ball_1, board_list=self.board_list)

        self.check_for_game_ended()
        self.update_counters()

        return

    def update_game(self):

        if not self.finished_game:

            if not self.wait:
                for event in pg.event.get():

                    # Check dragging
                    if event.type == pg.MOUSEBUTTONDOWN:
                        if event.button == 1:
                            for ball in self.max_balls_list:
                                if ball.is_colliding(event.pos):
                                    ball.dragging = True
                                    ball.previous_pos = ball.pos
                                    mouse_x, mouse_y = event.pos
                                    self.offset_x = ball.pos[0] - mouse_x
                                    self.offset_y = ball.pos[1] - mouse_y


                    elif event.type == pg.MOUSEBUTTONUP:
                        if event.button == 1:
                            for ball in self.max_balls_list:
                                if ball.dragging is True:
                                    relocation = ball.update_to_new_board(self.board_list, self.min_balls_list, self.max_balls_list)

                                    # If relocation was sucessfull send a message to the game manager
                                    if relocation:
                                        # Check for game ended
                                        rospy.loginfo("Robot Turn = " + str(self.robot_turn))
                                        rospy.loginfo("Child Turn = " + str(self.child_turn))
                                        self.update_counters()
                                        self.check_for_game_ended()
                                        self.send_information_to_manager()

                                    # Else the child needs to try again
                                    else:
                                        ball.move_ball(pos=ball.previous_pos)

                                ball.dragging = False

                    elif event.type == pg.MOUSEMOTION:
                        for ball in self.max_balls_list:
                            if ball.is_colliding(event.pos):
                                if ball.dragging:
                                    mouse_x, mouse_y = event.pos
                                    new_pos = [mouse_x + self.offset_x, mouse_y + self.offset_y]
                                    ball.move_ball(pos=new_pos)

            # update counters
            self.update_counters()

            # Check for game ended
            self.check_for_game_ended()

        # update the display
        self.update_display(ttt=self.ttt, board=self.board)
        return


    def check_for_game_ended(self):

        if self.robot_turn and self.min_score_board.get_score() < self.max_score_board.get_score() and not self.finished_game:
            rospy.loginfo("[GAME INTERFACE] Robot wins the game")
            self.wait = True
            self.finished_game = True
            self.child_win = False

        if self.child_turn and self.min_score_board.get_score() < self.max_score_board.get_score() and not self.finished_game:
            rospy.loginfo("[GAME INTERFACE] Child wins the game")
            self.wait = True
            self.finished_game = True
            self.child_win = True



    def update_counters(self):

        # Update MAX counter
        max_score = 0
        for ball in self.max_balls_list:
            max_score += ball.get_score()

        self.max_score_board.set_score(max_score)

        # Update Min Counter
        min_score = 0
        for ball in self.min_balls_list:
            min_score += ball.get_score()

        self.min_score_board.set_score(min_score)

        return



    def initBoard(self, ttt):
        # initialize the board and return it as a variable
        # ---------------------------------------------------------------
        # ttt : a properly initialized pyGame display variable

        # set up the background surface
        self.background = pg.Surface (ttt.get_size())
        self.background = self.background.convert()
        self.background.fill ((250, 250, 250))



        # draw the grid lines
        # vertical lines...
        # pg.draw.rect (background, (0,0,0), self.min_score_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_800_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_400_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_200_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_100_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_80_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_40_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_20_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_10_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_8_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_4_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_2_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.box_1_rect, 2)
        # pg.draw.rect (background, (0,0,0), self.max_score_rect, 2)
        
        
        # Draw the values

        # return the board
        return self.background

    def update_display (self, ttt, board):

        ttt.fill((0, 0, 0))
        self.background = pg.Surface(ttt.get_size())
        self.background = self.background.convert()
        self.background.fill((250, 250, 250))

        self.min_score_board.draw(self.background)
        self.max_score_board.draw(self.background)

        self.box_800_board.draw(self.background)
        self.box_400_board.draw(self.background)
        self.box_200_board.draw(self.background)
        self.box_100_board.draw(self.background)

        self.box_80_board.draw(self.background)
        self.box_40_board.draw(self.background)
        self.box_20_board.draw(self.background)
        self.box_10_board.draw(self.background)

        self.box_8_board.draw(self.background)
        self.box_4_board.draw(self.background)
        self.box_2_board.draw(self.background)
        self.box_1_board.draw(self.background)

        self.circle_min_ball_0.draw(self.background)
        self.circle_min_ball_1.draw(self.background)
        self.circle_max_ball_0.draw(self.background)
        self.circle_max_ball_1.draw(self.background)

        # If the game has ended and the child wins:
        if self.finished_game and self.child_win:

            self.child_win_board.draw(self.background)
            #self.play_again_board.draw(self.background)

        elif self.finished_game and not self.child_win:

            self.robot_win_board.draw(self.background)

        ttt.blit(self.background, (0, 0))
        pg.display.update()
        pg.display.update()




if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    rospy.loginfo("[Game Interface] Node is running...")
    gi = GameInterface()
    rate = rospy.Rate(80.0)
    while not rospy.is_shutdown():
        gi.update_game()
        rate.sleep()















