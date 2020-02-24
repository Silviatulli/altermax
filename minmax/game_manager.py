#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import sys
from src.utils import *
from minmax.srv import *
import pdb

NODE_NAME = 'minmax_game_module'

"""
Module responsible for the Game
"""

class GameManager:

    def __init__(self):

        # Game manager Variable
        self.finished_game = None
        self.wait = None
        self.robot_turn = None
        self.child_turn = None
        self.robot_id = None
        self.child_id = None
        self.minimizer_ball_0 = None
        self.minimizer_ball_1 = None
        self.maximizer_ball_0 = None
        self.maximizer_ball_1 = None
        self.optimal_robot_decision = None
        self.process = None


        # Services
        self.game_to_manager_service = rospy.Service('game_to_manager_service', GameState, self.handle_game_to_manager)
        rospy.wait_for_service('decision_service')
        rospy.wait_for_service('robot_talk_service')
        rospy.wait_for_service('manager_to_game_service')
        try:
            self.decision_proxy = rospy.ServiceProxy('decision_service', Decision)
            self.robot_talk_proxy = rospy.ServiceProxy('robot_talk_service', RobotTalk)
            self.manager_to_game_proxy = rospy.ServiceProxy('manager_to_game_service', GameState)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


        # Setup Game
        self.start_game()


    def start_game(self):

        # Game manager Variable
        self.finished_game = False
        self.wait = False
        self.robot_turn = False
        self.child_turn = True
        self.robot_id = player_type_dic['min']
        self.child_id = player_type_dic['max']
        self.minimizer_ball_0 = 200
        self.minimizer_ball_1 = 800
        self.maximizer_ball_0 = 1
        self.maximizer_ball_1 = 4
        self.optimal_robot_decision = True
        self.process = False

        # Send to Game the information
        game_resp = self.send_state_to_game()
        return game_resp

    def ask_robot_to_talk(self, speech_type=0):

        robot_talk_request = RobotTalkRequest()
        robot_talk_request.speech_type = speech_type
        robot_talk_resp = self.robot_talk_proxy.call(robot_talk_request)
        return robot_talk_resp


    def ask_decision(self):

        decision_request = DecisionRequest()
        decision_request.optimal = self.optimal_robot_decision
        decision_request.maximizer_ball_0 = self.maximizer_ball_0
        decision_request.maximizer_ball_1 = self.maximizer_ball_1
        decision_request.minimizer_ball_0 = self.minimizer_ball_0
        decision_request.minimizer_ball_1 = self.minimizer_ball_1

        # If it is an optimal solution we do not ask for generation of explanation
        if self.optimal_robot_decision:
            decision_request.generate_explanation = False
        else:
            decision_request.generate_explanation = True  # THIS IS WHY IT WAS NOT GENERATING EXPLANATION WITH THE ROBOT!

        rospy.loginfo("[Game Manager] Sending information to Decision Manager")
        rospy.loginfo("[Game Manager] Optimal Decision = " + str(decision_request.optimal))
        rospy.loginfo("[Game Manager] Max Ball 0 = " + str(decision_request.maximizer_ball_0))
        rospy.loginfo("[Game Manager] Max Ball 1 = " + str(decision_request.maximizer_ball_1))
        rospy.loginfo("[Game Manager] Min Ball 0 = " + str(decision_request.minimizer_ball_0))
        rospy.loginfo("[Game Manager] Min Ball 1 = " + str(decision_request.minimizer_ball_1))

        # Send to the decision manager and get the response
        decision_resp = self.decision_proxy.call(decision_request)
        return decision_resp



    def send_state_to_game(self, finished_game=None, wait=None, robot_turn=None, child_turn=None, min_ball_0=None,
                           min_ball_1=None, max_ball_0=None, max_ball_1=None):

        # Send to Game the information
        if wait is None:
            game_request = GameStateRequest()
            game_request.finished_game = self.finished_game
            game_request.wait = self.wait
            game_request.robot_turn = self.robot_turn
            game_request.child_turn = self.child_turn
            game_request.maximizer_ball_0 = self.maximizer_ball_0
            game_request.maximizer_ball_1 = self.maximizer_ball_1
            game_request.minimizer_ball_0 = self.minimizer_ball_0
            game_request.minimizer_ball_1 = self.minimizer_ball_1

        else:
            game_request = GameStateRequest()
            game_request.finished_game = finished_game
            game_request.wait = wait
            game_request.robot_turn = robot_turn
            game_request.child_turn = child_turn
            game_request.maximizer_ball_0 = max_ball_0
            game_request.maximizer_ball_1 = max_ball_1
            game_request.minimizer_ball_0 = min_ball_0
            game_request.minimizer_ball_1 = min_ball_1

        rospy.loginfo("[Game Manager] Sending information to Game Interface")
        rospy.loginfo("[Game Manager] Finished Game = " + str(game_request.finished_game))
        rospy.loginfo("[Game Manager] Wait = " + str(game_request.wait))
        rospy.loginfo("[Game Manager] Robot Turn = " + str(game_request.robot_turn))
        rospy.loginfo("[Game Manager] Child Turn = " + str(game_request.child_turn))
        rospy.loginfo("[Game Manager] Max Ball 0 = " + str(game_request.maximizer_ball_0))
        rospy.loginfo("[Game Manager] Max Ball 1 = " + str(game_request.maximizer_ball_1))
        rospy.loginfo("[Game Manager] Min Ball 0 = " + str(game_request.minimizer_ball_0))
        rospy.loginfo("[Game Manager] Min Ball 1 = " + str(game_request.minimizer_ball_1))

        # Call the manager to game service and wait for response
        game_response = self.manager_to_game_proxy.call(game_request)
        return game_response.success


    def handle_game_to_manager(self, req):

        rospy.loginfo("[Game Manager] Receiving information from Game Interface")

        # Read Request and update variables
        self.finished_game = req.finished_game
        self.wait = req.wait
        self.robot_turn = req.robot_turn
        self.child_turn = req.child_turn
        self.maximizer_ball_0 = req.maximizer_ball_0
        self.maximizer_ball_1 = req.maximizer_ball_1
        self.minimizer_ball_0 = req.minimizer_ball_0
        self.minimizer_ball_1 = req.minimizer_ball_1

        rospy.loginfo("[Game Manager] Finished Game = " + str(self.finished_game))
        rospy.loginfo("[Game Manager] Wait = " + str(self.wait))
        rospy.loginfo("[Game Manager] Robot Turn = " + str(self.robot_turn))
        rospy.loginfo("[Game Manager] Child Turn = " + str(self.child_turn))
        rospy.loginfo("[Game Manager] Max Ball 0 = " + str(self.maximizer_ball_0))
        rospy.loginfo("[Game Manager] Max Ball 1 = " + str(self.maximizer_ball_1))
        rospy.loginfo("[Game Manager] Min Ball 0 = " + str(self.minimizer_ball_0))
        rospy.loginfo("[Game Manager] Min Ball 1 = " + str(self.minimizer_ball_1))


        # Tell the game manager we need to process information
        self.process = True
        rospy.loginfo("[Game Manager] Processing Information = " + str(self.process))

        # Send Response that we updated the variables
        gstate_resp = GameStateResponse()
        gstate_resp.success = True
        return gstate_resp


    def process_information(self):

        if not self.finished_game:

            if not self.process:
                return
            else:

                rospy.loginfo("[Game Manager] Processing information")
                # If it was the robot turn that ended, then we change to the child's turn
                if self.robot_turn:
                    self.robot_to_child_turn()

                # if it was the child turn that ended, we need to change to the robot's turn
                if self.child_turn:
                    self.child_to_robot_turn()

                self.process = False
        else:
            rospy.loginfo("[Game Manager] Finished Game")

    def robot_to_child_turn(self):

        rospy.loginfo("[Game Manager] Changing robot -> child turn")

        # Change robot to child turn
        self.child_turn = True
        self.robot_turn = False

        # Send message to game
        game_resp = self.send_state_to_game()
        if not game_resp:
            rospy.loginfo("[Game Manager] ERROR! Could not change robot -> child turn.")
        return


    def child_to_robot_turn(self):

        rospy.loginfo("[Game Manager] Changing child -> robot turn")
        rospy.loginfo("[Game Manager] Asking Game to freeze all balls.")

        # First thing, ask the game to freeze all balls
        self.wait = True
        game_resp = self.send_state_to_game()

        if not game_resp:
            rospy.loginfo("[Game Manager] ERROR! Could not freeze all balls.")

        # Next, ask the decision module, where should we move the balls! So we make a request
        decision_resp = self.ask_decision()

        # Read the response
        ball_to_move = decision_resp.ball
        new_ball_score = decision_resp.new_ball_score

        # if the ball to move is the ball 0, we update its score
        if ball_to_move == 0:
            self.minimizer_ball_0 = new_ball_score
        else:
            self.minimizer_ball_1 = new_ball_score

        # Send to game the new information
        self.wait = False
        self.robot_turn = True
        self.child_turn = False
        game_resp = self.send_state_to_game()

        if not game_resp:
            rospy.loginfo("[Game Manager]  ERROR! Could not Move balls.")

        # If we don't need to generate explanation it is finished
        if self.optimal_robot_decision:
            rospy.loginfo("[Game Manager] Optimal Decision was decided. Next turn it will be sub-optimal")
            self.optimal_robot_decision = False  # NEXT TURN IT WONT BE OPTIMAL

        # If not, we need to generate explanation
        else:
            rospy.loginfo("[Game Manager] Sub Optimal Decision was decided. Next turn it will be optimal.")
            self.optimal_robot_decision = True  # NEXT TURN IT WILL BE OPTIMAL

            # First thing, ask the game to freeze all balls
            rospy.loginfo("[Game Manager] Asking game interface to freeze balls.")
            self.wait = True
            game_resp = self.send_state_to_game()

            if not game_resp:
                rospy.loginfo("[Game Manager] ERROR! Could not Freeze all balls.")


            # Next ask the robot to talk
            rospy.loginfo("[Game Manager] Asking robot to generate explanation.")

            robot_resp = self.robot_talk_proxy.call(robot_speech_type_dic['explanation'])
            if not robot_resp:
                rospy.loginfo("[Game Manager] ERROR! Could not make robot explain.")

        # Next change to child's turn:
        self.wait = False
        self.robot_to_child_turn()
        return


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    gm = GameManager()
    rospy.loginfo("[Game Manager] Node is running...")
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        gm.process_information()
        rate.sleep()
