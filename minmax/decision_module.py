#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import sys
from src.utils import *
from minmax.srv import *

NODE_NAME = 'minmax_decision_module'

"""
Module responsible for the Robot's decision


"""

class DecisionManager:

    def __init__(self):

        # Game Variables
        self.max_depth = 3

        # Explanation Variables
        self.action = None
        self.ball = None
        self.score = None
        self.new_ball_score = None
        self.best_action = None
        self.best_ball = None
        self.best_score = None
        self.best_new_ball_score = None

        # Services
        self.decision_service = rospy.Service('decision_service', Decision, self.handle_decision_service)
        self.robot_talk_service = rospy.Service('robot_talk_service', RobotTalk, self.handle_robot_talk_service)

        rospy.wait_for_service('plan_service')
        try:
            self.plan_proxy = rospy.ServiceProxy('plan_service', Plan)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        rospy.wait_for_service('robot_explanation_service')
        try:
            self.robot_explanation_proxy = rospy.ServiceProxy('robot_explanation_service', RobotExplanation)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        rospy.loginfo("[")


    def handle_decision_service(self, req):

        # Reads the request
        optimal_action = req.optimal
        generate_explanation = req.generate_explanation
        min_ball_0 = req.minimizer_ball_0
        min_ball_1 = req.minimizer_ball_1
        max_ball_0 = req.maximizer_ball_0
        max_ball_1 = req.maximizer_ball_1


        # If it is an optimal action, we just get the best action:
        if optimal_action:

            # Create service message
            plan_request = PlanRequest()
            plan_request.policy = policy_dic['best']
            plan_request.depth = self.max_depth
            plan_request.maximizer_ball_0 = max_ball_0
            plan_request.maximizer_ball_1 = max_ball_1
            plan_request.minimizer_ball_0 = min_ball_0
            plan_request.minimizer_ball_1 = min_ball_1

            # Call the planning service and wait for response
            plan_resp = self.plan_proxy.call(plan_request)

            # Process response
            self.ball = plan_resp.ball
            self.action = plan_resp.action
            self.new_ball_score = plan_resp.new_ball_score
            self.score = plan_resp.score

            # Send to Game Manager to ball to move and the action
            decision_resp = DecisionResponse()
            decision_resp.ball = self.ball
            decision_resp.action = self.action
            decision_resp.new_ball_score = self.new_ball_score
            return decision_resp

        # If it is not an optimal action, USE THE RANDOM ACTION (CHANGE IN THE FUTURE)
        else:

            # We get a randomly selected action, except the best
            # Create service message
            plan_request = PlanRequest()
            plan_request.policy = policy_dic['random']
            plan_request.depth = self.max_depth
            plan_request.maximizer_ball_0 = max_ball_0
            plan_request.maximizer_ball_1 = max_ball_1
            plan_request.minimizer_ball_0 = min_ball_0
            plan_request.minimizer_ball_1 = min_ball_1

            # Call the planning service and wait for response
            plan_resp = self.plan_proxy.call(plan_request)

            # Process response
            self.ball = plan_resp.ball
            self.action = plan_resp.action
            self.new_ball_score = plan_resp.new_ball_score
            self.score = plan_resp.score


            # If we want to generate the explanation, we also get the best policy
            if generate_explanation:

                # We get a randomly selected action, except the best
                # Create service message
                plan_request = PlanRequest()
                plan_request.policy = policy_dic['best']
                plan_request.depth = self.max_depth
                plan_request.maximizer_ball_0 = max_ball_0
                plan_request.maximizer_ball_1 = max_ball_1
                plan_request.minimizer_ball_0 = min_ball_0
                plan_request.minimizer_ball_1 = min_ball_1

                # Call the planning service and wait for response
                plan_resp = self.plan_proxy.call(plan_request)

                # Process response
                self.best_ball = plan_resp.ball
                self.best_action = plan_resp.action
                self.best_score = plan_resp.score
                self.best_new_ball_score = plan_resp.new_ball_score

            # Send to Game Manager to ball to move and the action
            decision_resp = DecisionResponse()
            decision_resp.ball = self.ball
            decision_resp.action = self.action
            decision_resp.new_ball_score = self.new_ball_score
            return decision_resp



    def handle_robot_talk_service(self, req):

        # Read the request:
        speech_type = req.speech_type

        # If we want the robot to explain the move
        if speech_type == 0:
            # Create service message to send to robot
            rexplan_request = RobotExplanationRequest()
            rexplan_request.depth = self.max_depth
            rexplan_request.ball = self.ball
            rexplan_request.action = self.action
            rexplan_request.score = self.score
            rexplan_request.best_ball = self.best_ball
            rexplan_request.best_action = self.best_action
            rexplan_request.best_score = self.best_score

            # Call the robot explanation service and wait for response
            rexplan_resp = self.robot_explanation_proxy.call(rexplan_request)

        # Respond to service
        rtalk_resp = RobotTalkResponse()
        rtalk_resp.success = rexplan_resp.success
        return rtalk_resp


    def clean_explanation_variables(self):
        self.action = None
        self.ball = None
        self.score = None
        self.best_action = None
        self.best_ball = None
        self.best_score = None
        self.new_ball_score = None
        return

if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    dm = DecisionManager()
    rospy.loginfo("[Decision Manager] Node is running...")
    rospy.spin()















