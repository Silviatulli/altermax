import numpy as np
import random
import rospy

GAME_MAX_CELL_X = 5
GAME_MAX_CELL_Y = 1
action_dic = {'up':0, 'down':1, 'left':2, 'right':3, 'up_left':4, 'up_right':5, 'down_left':6, 'down_right':7}
player_type_dic = {'min':0, 'max':1}
ball_initial_pos = {0:{0:[GAME_MAX_CELL_X, 1], 1:[GAME_MAX_CELL_X, 0]}, 1:{0:[0, 1], 1:[0, 0]}}
policy_dic = {'best':0, 'random':1, 'worst':2}
robot_speech_type_dic = {'explanation': 0}
board = np.array([[1, 2, 10, 20, 100, 200], [4, 8, 40, 80, 400, 800]]).T


def get_action_name(action_value):
    for action_name, value in action_dic.items():
        if value == action_value:
            return action_name


def get_player_name(player_value):
    for player_name, value in player_type_dic.items():
        if value == player_value:
            return player_name

# define the cell
class Board(object):
    def __init__(self):
        self.board = board

    # Get coordinates from the board by the selected value
    def get_pos(self, value):
        pos = np.where(self.board == value)
        return [int(pos[0]), int(pos[1])]

    # Get value in board from posiion
    def get_value(self, pos):
        return self.board[pos[0], pos[1]]


# define the players
class Player(object):
    def __init__(self, player_type, ballsAmount):
        self.score = 0
        self.player_type = player_type
        self.balls = []

        for i in range(ballsAmount):
            newBall = Ball(id=i, player_id=player_type, pos=ball_initial_pos[player_type][i])
            self.balls.append(newBall)


# define the balls
class Ball(object):
    def __init__(self, id, player_id, pos):
        self.id = id
        self.player_id = player_id
        self.pos = pos

    def move(self, action):

        if self.player_id == player_type_dic['max']:

            if action == 0:   # UP
                if self.pos[1] < GAME_MAX_CELL_Y:
                    self.pos[1] += 1
                else:
                    # print('invalid action - Cannot go up')
                    return None

            elif action == 1:   # DOWN
                return None

            elif action == 2:   # LEFT
                if self.pos[0] < GAME_MAX_CELL_X:
                    self.pos[0] += 1
                else:
                    # print('invalid action - Cannot go left')
                    return None

            elif action == 3:  # RIGHT
                return None

            elif action == 4:  # UP LEFT
                if self.pos[1] < GAME_MAX_CELL_Y and self.pos[0] < GAME_MAX_CELL_X:
                    self.pos[1] += 1
                    self.pos[0] += 1
                else:
                    # print('invalid action - Cannot go up_left')
                    return None

            elif action == 5:  # UP RIGHT
                if self.pos[1] < GAME_MAX_CELL_Y and self.pos[0] < GAME_MAX_CELL_X and \
                        board[self.pos[0]+1, self.pos[1]+1] > board[self.pos[0], self.pos[1]]:  # TODO CHANGED THIS!!!!!
                    self.pos[1] += 1
                    self.pos[0] += 1
                else:
                    # print('invalid action - Cannot go up_left')
                    return None


            elif action == 6:  # DOWN LEFT
                if self.pos[1] > 0 and self.pos[0] < GAME_MAX_CELL_X and\
                        board[self.pos[0]+1, self.pos[1]-1] > board[self.pos[0], self.pos[1]]:  # TODO CHANGED THIS!!!!!
                    self.pos[0] += 1
                    self.pos[1] -= 1
                else:
                    # print('invalid action - Cannot go down_left')
                    return None

            elif action == 7:
                return None

            else:
                print('invalid action')
                return None

        # FOR MINIMIZER PLAYERS
        else:

            if action == 0:   # UP
                return None


            elif action == 1:  # DOWN
                if self.pos[1] > 0:
                    self.pos[1] -= 1
                else:
                    # print('invalid action - Cannot go down')
                    return None

            elif action == 2:   # LEFT
                return None

            elif action == 3:  # RIGHT
                if self.pos[0] > 0:
                    self.pos[0] -= 1
                else:
                    # print('invalid action - Cannot go right')
                    return None

            elif action == 4: # UP LEFT
                return None

            elif action == 5:  # UP RIGHT
                if self.pos[1] < GAME_MAX_CELL_Y and self.pos[0] > 0 and \
                        board[self.pos[0]-1, self.pos[1]+1] < board[self.pos[0], self.pos[1]]:  # TODO CHANGED THIS!!!!!
                    self.pos[0] -= 1
                    self.pos[1] += 1
                else:
                    # print('invalid action - Cannot go up_right')
                    return None

            elif action == 6:  # DOWN LEFT
                if self.pos[1] > 0 and self.pos[0] < GAME_MAX_CELL_X and \
                        board[self.pos[0]+1, self.pos[1]-1] < board[self.pos[0], self.pos[1]]: # TODO CHANGED THIS!!!!!
                    self.pos[0] += 1
                    self.pos[1] -= 1
                else:
                    # print('invalid action - Cannot go down_left')
                    return None
            elif action == 7:
                if self.pos[1] > 0 and self.pos[0] > 0:
                    self.pos[0] -= 1
                    self.pos[1] -= 1
                else:
                    # print('invalid action - Cannot go down_right')
                    return None
            else:
                print('invalid action')
                return None

        return self.pos

class Node(object):
    def __init__(self, min_ball0, min_ball1, max_ball0, max_ball1, last_action, last_ball_moved, last_action_type,
                 depth, parent):

        # State Variables
        self.min_ball_scores = [min_ball0, min_ball1]
        self.max_ball_scores = [max_ball0, max_ball1]
        self.min_total_score = min_ball0 + min_ball1
        self.max_total_score = max_ball0 + max_ball1
        self.parent = parent
        self.last_action_type = last_action_type
        self.depth = depth
        self.children = []
        self.last_action = last_action
        self.last_ball_moved = last_ball_moved

    def addChild(self, child):
        self.children.append(child)  # this add a child to the children list

    def check_balls_position(self, new_node, action_type):

        # Check if both balls of new node are in the same position
        if action_type == player_type_dic['max']:
            if new_node.max_ball_scores[0] == new_node.max_ball_scores[1]:
                return False
            # if new_node.max_ball_scores[0] == self.min_ball_scores[0] or new_node.max_ball_scores[0] == self.min_ball_scores[1]:
            #  return False
            # elif new_node.max_ball_scores[1] == self.min_ball_scores[0] or new_node.max_ball_scores[1] == self.min_ball_scores[1]:
            #  return False
            else:
                return True
            # For each of the new node balls, check if its position is the same as any of the parents balls position
        if action_type == player_type_dic['min']:
            if new_node.min_ball_scores[0] == new_node.min_ball_scores[1]:
                return False
            # if new_node.min_ball_scores[0] == self.max_ball_scores[0] or new_node.min_ball_scores[0] == self.max_ball_scores[1]:
            #  return False
            # elif new_node.min_ball_scores[1] == self.max_ball_scores[0] or new_node.min_ball_scores[1] == self.max_ball_scores[1]:
            #  return False
            else:
                return True
        else:
            return True

    def expand(self, board, action_type):
        # For each ball we try all possible actions
        if action_type == player_type_dic['max']:

            for i in range(len(self.max_ball_scores)):
                # import pdb;pdb.set_trace()
                # Get position of ball from its score
                ball_pos = board.get_pos(value=self.max_ball_scores[i])

                # Lets try all action and check if they are valid
                for k in range(len(action_dic)):
                    ball = Ball(id=i, player_id=player_type_dic['max'], pos=ball_pos[:])
                    new_pos = ball.move(action=k)
                    # If the action is invalid, we forget it
                    if new_pos is None:
                        # print("Action is invalid")
                        continue
                    # If the action is valid, we create a child node
                    else:
                        new_ball_score = board.get_value(pos=new_pos)

                        if i == 0:

                            new_node = Node(min_ball0=self.min_ball_scores[0],
                                            min_ball1=self.min_ball_scores[1],
                                            max_ball0=new_ball_score,
                                            max_ball1=self.max_ball_scores[1],
                                            last_action=k,
                                            last_ball_moved=i,
                                            last_action_type=action_type,
                                            depth=self.depth + 1,
                                            parent=self)


                        else:

                            new_node = Node(min_ball0=self.min_ball_scores[0],
                                            min_ball1=self.min_ball_scores[1],
                                            max_ball0=self.max_ball_scores[0],
                                            max_ball1=new_ball_score,
                                            last_action=k,
                                            last_ball_moved=i,
                                            last_action_type=action_type,
                                            depth=self.depth + 1,
                                            parent=self)

                        # Check if the new node has both balls in the same position or in the parents balls position
                        if self.check_balls_position(new_node=new_node, action_type=action_type):
                            self.addChild(new_node)
                        else:
                            # Append new node to the children list
                            continue
            return
        if action_type == player_type_dic['min']:

            for i in range(len(self.min_ball_scores)):
                # import pdb;pdb.set_trace()
                # Get position of ball from its score
                ball_pos = board.get_pos(value=self.min_ball_scores[i])

                # Lets try all action and check if they are valid
                for k in range(len(action_dic)):
                    ball = Ball(id=i, player_id=player_type_dic['min'], pos=ball_pos[:])
                    new_pos = ball.move(action=k)
                    # If the action is invalid, we forget it
                    if new_pos is None:
                        # print("Action is invalid")
                        continue
                    # If the action is valid, we create a child node
                    else:
                        new_ball_score = board.get_value(pos=new_pos)

                        if i == 0:

                            new_node = Node(min_ball0=new_ball_score,
                                            min_ball1=self.min_ball_scores[1],
                                            max_ball0=self.max_ball_scores[0],
                                            max_ball1=self.max_ball_scores[1],
                                            last_action=k,
                                            last_ball_moved=i,
                                            last_action_type=action_type,
                                            depth=self.depth + 1,
                                            parent=self)


                        else:

                            new_node = Node(min_ball0=self.min_ball_scores[0],
                                            min_ball1=new_ball_score,
                                            max_ball0=self.max_ball_scores[0],
                                            max_ball1=self.max_ball_scores[1],
                                            last_action=k,
                                            last_ball_moved=i,
                                            last_action_type=action_type,
                                            depth=self.depth + 1,
                                            parent=self)

                        # Check if the new node has both balls in the same position or in the parents balls position
                        if self.check_balls_position(new_node=new_node, action_type=action_type):
                            self.addChild(new_node)
                        else:
                            # Append new node to the children list
                            continue

            return



class DecisionTree:
    def __init__(self, root, board):
        self.root = root
        self.openNodes = []
        self.closeNodes = []
        self.board = board
        self.max_scores = []
        self.min_scores = []
        self.depth_max_nodes = []
        self.win_max_nodes = []
        self.win_min_nodes = []

    def populateTree(self, max_depth=1):

        if self.root is None:
            rospy.loginfo['[Decision Module] No Root selected..']
            return False

        while len(self.openNodes) != 0:

            # Pop first node in open Nodes List
            node = self.openNodes.pop(0)

            # Check if we can continue expansion
            if self.check_node_expansion(node, max_depth):

                if node.last_action_type == 0:
                    node.expand(board=self.board, action_type=1)
                else:
                    node.expand(board=self.board, action_type=0)

                # append the children of the node to the open nodes
                self.openNodes.extend(node.children)

                self.closeNodes.append(node)

            else:
                self.closeNodes.append(node)

        # If there are no nodes at max depth (because the game ends before that) this might blow up!
        if len(self.depth_max_nodes) == 0:
            for i in range(max_depth, 0 , -2):

                self.nodes_less_depth = []

                # Check the nodes at depth i
                for node in self.closeNodes:
                    if node.depth == i:
                        self.nodes_less_depth.append(node)

                # Check if the list is stil empty, if it is not we break the loop cycle
                if len(self.nodes_less_depth) != 0:
                    self.depth_max_nodes = self.nodes_less_depth
                    break
                else:
                    continue
        return







    def check_node_expansion(self, node, max_depth):

        continue_expansion = True

        # Check if the game ended
        if node.min_total_score < node.max_total_score:
            continue_expansion = False

            # see who won
            if node.last_action_type == player_type_dic['max']:
                self.win_max_nodes.append(node)
            else:
                self.win_min_nodes.append(node)

        # Check if we reached maximum depth
        if node.depth >= max_depth:

            continue_expansion = False

            if node.depth == max_depth:
                self.depth_max_nodes.append(node)

        return continue_expansion

    def set_root(self, root_node):
        self.root = root_node
        self.openNodes.append(root_node)
        return

    def clean_tree(self):
        # Cleans all variables
        self.openNodes = []
        self.closeNodes = []
        self.max_scores = []
        self.min_scores = []
        self.depth_max_nodes = []
        self.win_max_nodes = []
        self.win_min_nodes = []
        self.root = None
        return

    def get_depth_max_nodes(self):
        return self.depth_max_nodes

    def get_maximizer_win_nodes(self):
        return self.win_max_nodes

    def get_minimizer_win_nodes(self):
        return self.win_min_nodes
