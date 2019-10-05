#!/usr/bin/env python
import rospy
import sys
import copy as cp
from src.utils import *
from minmax.srv import *

NODE_NAME = 'minmax_planning_manager'

"""
Module responsible for the Robot's planning


"""



class PlanningManager:

    def __init__(self):
        self.root = None
        self.board = Board()
        self.decision_tree = DecisionTree(root=self.root, board=self.board)
        self.plan_service = rospy.Service('plan_service', Plan, self.handle_plan_service)


    def get_best_action(self, player_type):

        # Reset Variables
        best_node = None
        best_node_value = None
        depth_max_nodes = self.decision_tree.get_depth_max_nodes()
        rospy.loginfo("Number of depth max nodes = " + str(
            len(self.decision_tree.get_depth_max_nodes())) + "for best selection of action")


        # If the decision is for the max player, we want the node with the maximum value
        if player_type == player_type_dic['max']:

            # TODO: If we have nodes where the player wins, that is the best solution obviously
            win_nodes = self.decision_tree.get_maximizer_win_nodes()
            rospy.loginfo("Number of winning nodes = " + str(
                len(self.decision_tree.get_maximizer_win_nodes())) + "for best selection of action")

            if len(win_nodes) != 0:  # We are going to select randomly a win node -- NOT THE BEST SOLUTION BUT IT WILL DO FOR NOW
                best_node = random.choice(win_nodes)

            else:
                # Iterate over the nodes in the max depth list and find the node with the maximum val()
                best_node_value = 0.0

                for node in depth_max_nodes:

                    if node.max_total_score > best_node_value:
                        best_node = node
                        best_node_value = node.max_total_score

        # If the decision is for the max player, we want the node with the maximum value
        elif player_type == player_type_dic['min']:

            # TODO: If we have nodes where the player wins, that is the best solution obviously
            win_nodes = self.decision_tree.get_minimizer_win_nodes()
            rospy.loginfo("Number of winning nodes = " + str(
                len(self.decision_tree.get_minimizer_win_nodes())) + "for best selection of action")

            if len(win_nodes) != 0:  # We are going to select randomly a win node -- NOT THE BEST SOLUTION BUT IT WILL DO FOR NOW
                best_node = random.choice(win_nodes)

            else:
                # Iterate over the nodes and find the node with the minimum value
                best_node_value = float('inf')

                for node in depth_max_nodes:

                    if node.min_total_score < best_node_value:
                        best_node = node
                        best_node_value = node.min_total_score

        else:
            rospy.loginfo("[Planning Manager] Error! Wrong player type selected")
            return None, None, None

        # We need to go back to depth=1 to decide which is the best action
        node = best_node
        node_tree = []
        node_tree.append(node)
        while node.depth != 1:
            node = node.parent
            node_tree.append(node)

        # Now we have the node at depth = 1
        action_to_take = node.last_action
        ball_to_move = node.last_ball_moved
        new_ball_score = node.min_ball_scores[ball_to_move]

        return ball_to_move, action_to_take, new_ball_score, best_node.min_total_score  # WE ONLY DO THIS FOR THE MIN (ROBOT) PLAYER!!!!



    def get_worst_action(self, player_type):

        rospy.loginfo("Number of depth max nodes = " + str(len(self.decision_tree.get_depth_max_nodes())) + "for worst selection of action")
        # Reset Variables
        worst_node = None
        worst_node_value = None
        depth_max_nodes = self.decision_tree.get_depth_max_nodes()

        # If the decision is for the max player, we want the node with the maximum value
        if player_type == player_type_dic['max']:

            # Iterate over the nodes and find the node with the maximum value
            worst_node_value = float('inf')

            for node in depth_max_nodes:

                if node.max_total_score < worst_node_value:
                    worst_node = node
                    worst_node_value = node.max_total_score
        # If the decision is for the max player, we want the node with the maximum value
        elif player_type == player_type_dic['min']:

            # pdb.set_trace()

            # Iterate over the nodes and find the node with the minimum value
            worst_node_value = 0.0

            for node in depth_max_nodes:

                if node.min_total_score > worst_node_value:
                    worst_node = node
                    worst_node_value = node.min_total_score

        else:
            rospy.loginfo("[Planning Manager] Error! Wrong player type selected")
            return None, None, None

        # We need to go back to depth=1 to decide which is the best action
        node = worst_node
        node_tree = []
        node_tree.append(node)
        while node.depth != 1:
            node = node.parent
            node_tree.append(node)

        # Now we have the node at depth = 1
        action_to_take = node.last_action
        ball_to_move = node.last_ball_moved
        new_ball_score = node.min_ball_scores[ball_to_move]

        return ball_to_move, action_to_take, new_ball_score, worst_node.min_total_score



    def get_random_action(self, player_type):

        # First copy the list of depth_max nodes
        depth_max_nodes = self.decision_tree.get_depth_max_nodes()
        new_depth_max_nodes = cp.deepcopy(depth_max_nodes)

        rospy.loginfo("Number of depth max nodes = " + str(len(new_depth_max_nodes))+ "for random selection of action")


        # Pop the best and the worst nodes
        best_node = None
        best_node_index = -1
        best_node_value = None
        worst_node_index = -1
        worst_node_value = None

        # FOR THE MAX PLAYER

        # If the decision is for the max player, we want the node with the maximum value
        if player_type == player_type_dic['max']:

            # Iterate over the nodes and find the node with the maximum value - BEST NODE to remove it from the random choices
            best_node_value = 0

            for i in range(len(new_depth_max_nodes)):

                if new_depth_max_nodes[i].max_total_score > best_node_value:
                    best_node_index = i
                    best_node_value = new_depth_max_nodes[i].max_total_score

            # Now we pop the max node out of the list
            best_node = new_depth_max_nodes.pop(best_node_index)

            # # Iterate over the nodes and find the node with the maximum value - WORST NODE
            # worst_node_value = float('inf')
            # for i in range(len(new_depth_max_nodes)):
            #
            #     if new_depth_max_nodes[i].max_total_score < worst_node_value:
            #         worst_node_index = i
            #         worst_node_value = new_depth_max_nodes[i].max_total_score
            #
            # # Now we pop the max node out of the list
            # _ = new_depth_max_nodes.pop(i)

            # NOW FOR THE MIN PLAYER

            # If the decision is for the max player, we want the node with the maximum value
        if player_type == player_type_dic['min']:

            # Iterate over the nodes and find the node with the maximum value - MAX NODE
            best_node_value = float('inf')
            for i in range(len(new_depth_max_nodes)):

                if new_depth_max_nodes[i].max_total_score < best_node_value:
                    best_node_index = i
                    best_node_value = new_depth_max_nodes[i].max_total_score

            # Now we pop the max node out of the list
            best_node = new_depth_max_nodes.pop(best_node_index)

            # # Iterate over the nodes and find the node with the maximum value - WORST NODE
            # worst_node_value = float('inf')
            # for i in range(len(new_depth_max_nodes)):
            #
            #     if new_depth_max_nodes[i].max_total_score > worst_node_value:
            #         worst_node_index = i
            #         worst_node_value = new_depth_max_nodes[i].max_total_score
            #
            # # Now we pop the max node out of the list
            # _ = new_depth_max_nodes.pop(i)

        else:
            rospy.loginfo("[Planning Manager] Error! Wrong player type selected")
            return None, None, None

        # Randomly decide with node we want - MUST HAVE DIFFERENT INITIAL ACTION

        # First determine what was the initial action of the best decision
        best_node_tree = []
        best_node_tree.append(best_node)
        while best_node.depth != 1:
            best_node = best_node.parent
            best_node_tree.append(best_node)
        best_node_initial_action = best_node.last_action
        best_node_initial_ball = best_node.last_ball_moved

        # Now we randomly decide a node with different initial action:
        different_initial_action = False
        action_to_take, ball_to_move, new_ball_score = None, None, None
        random_node = None
        while not different_initial_action:

            random_node_index = random.randrange(len(new_depth_max_nodes))
            random_node = new_depth_max_nodes[random_node_index]

            # We need to go back to depth=1 to decide which is the best action
            node = random_node

            node_tree = []
            node_tree.append(node)
            while node.depth != 1:
                node = node.parent
                node_tree.append(node)

            # Now we have the node at depth = 1
            action_to_take = node.last_action
            ball_to_move = node.last_ball_moved
            new_ball_score = node.min_ball_scores[ball_to_move]

            # If it is the same as the best node, but it is our last option, we take it
            if action_to_take == best_node_initial_action and ball_to_move == best_node_initial_ball and len(new_depth_max_nodes) == 1:
                # We finish searching
                different_initial_action = True

            # If it is the same as the best node but we stil have more options, we continue searching
            elif action_to_take == best_node_initial_action and ball_to_move == best_node_initial_ball:

                # We pop this node
                new_depth_max_nodes.pop(random_node_index)

                # We try again

            # If it is the same action but we move a different ball, we take it
            elif action_to_take == best_node_initial_action and ball_to_move != best_node_initial_ball:

                # We finish searching
                different_initial_action = True

            else:
                # We finish searching
                different_initial_action = True

        return ball_to_move, action_to_take, new_ball_score, random_node.min_total_score


    def handle_plan_service(self, req):

        # Read request
        policy = req.policy
        depth = req.depth
        min_ball_0 = req.minimizer_ball_0
        min_ball_1 = req.minimizer_ball_1
        max_ball_0 = req.maximizer_ball_0
        max_ball_1 = req.maximizer_ball_1


        # Create Root Node of the Decision Tree
        root_node = Node(min_ball0=min_ball_0,
                        min_ball1=min_ball_1,
                        max_ball0=max_ball_0,
                        max_ball1=max_ball_1,
                        last_action=player_type_dic['max'],  # Child is always the MAXIMIZER - CHANGE IN THE FUTURE
                        last_ball_moved=None,
                        last_action_type=None,
                        depth=0,
                        parent=None)

        # Set the root node of the Tree
        self.decision_tree.set_root(root_node=root_node)

        # Expand the tree
        self.decision_tree.populateTree(max_depth=depth)

        # Request selected policy
        ball, action, new_ball_score, score = None, None, None, None

        # Get best policy
        if policy == policy_dic['best']:
            ball, action, new_ball_score, score = self.get_best_action(player_type=player_type_dic['min'])

        # Get random policy
        elif policy == policy_dic['random']:
            ball, action, new_ball_score, score = self.get_random_action(player_type=player_type_dic['min'])

        # Get worst policy
        elif policy == policy_dic['worst']:
            ball, action, new_ball_score, score = self.get_worst_action(player_type=player_type_dic['min'])

        # Clean Tree
        self.decision_tree.clean_tree()

        # Write response to return
        response = PlanResponse()
        response.ball = ball
        response.action = action
        response.new_ball_score = new_ball_score
        response.score = score
        return response



if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    rospy.loginfo("[Planning Manager] Node is running...")
    pm = PlanningManager()
    rospy.spin()















