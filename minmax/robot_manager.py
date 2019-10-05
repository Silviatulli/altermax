#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import sys
from src.utils import *
from minmax.srv import *
from naoqi import ALProxy, ALBroker, ALModule

NODE_NAME = 'minmax_robot_manager'
pt_action_dic = {'up': "cima", 'down':"baixo", 'left':"a esquerda", 'right':"a direita", 'up_left':"cima e esquerda", 'up_right':"cima e direita",
                 'down_left':"baixo e esquerda", 'down_right':"baixo e direita"}



"""
Module responsible for the Robot's Conections and Behaviors


"""


class RobotManager:

    def __init__(self):

        # Explanation Variables
        self.depth = None
        self.ball = None
        self.action = None
        self.score = None
        self.best_ball = None
        self.best_action = None
        self.best_score = None
        self.num_turns = None
        self.explanation_text = None

        # Services
        self.robot_explanation_service = rospy.Service('robot_explanation_service', RobotExplanation,
                                                       self.handle_robot_explanation_service)

        #Robot Connection Variables and Services
        self.nao_IP = '192.168.1.100'
        self.nao_port = 9559

        self.myBroker = ALBroker("myBroker",
                             "0.0.0.0",  # listen to anyone
                             0,  # find a free port and use it
                             self.nao_IP,  # parent broker IP
                             9559)  # parent broker port)
        
        self.robot_communication = ALProxy("ALTextToSpeech", self.nao_IP, self.nao_port)
        self.robot_communication.setLanguage('Portuguese')
        self.robot_communication.setVolume(1)
        self.robot_communication.setParameter("speed", 70)
        
        #self.faceProxy = ALProxy("ALFaceDetection", self.nao_IP, self.nao_port)
        self.tracker = ALProxy("ALTracker", self.nao_IP, self.nao_port)
        self.motionProxy = ALProxy("ALMotion", self.nao_IP, self.nao_port)
        self.ledProxy = ALProxy("ALLeds", self.nao_IP, self.nao_port)
        self.postureProxy = ALProxy("ALRobotPosture", self.nao_IP, self.nao_port)
        
        # Setup Robot
        self.wake_robot()



    def handle_robot_explanation_service(self, req):

        # Read the request
        self.depth = req.depth
        self.ball = req.ball
        self.action = req.action
        self.score = req.score
        self.best_ball = req.best_ball
        self.best_action = req.best_action
        self.best_score = req.best_score

        # Count number of turns
        self.num_turns = int((self.depth - 1)/2.0)

        # Generate Explanation
        self.explanation_text = self.generate_explanation()

        # Send explanation text to the robot
        #self.robot_communication.say(self.explanation_text)
        print(self.explanation_text)

        # Clean explanation variables
        self.clean_explanation_variables()

        # Respond to service
        rexplan_resp = RobotExplanationResponse()
        rexplan_resp.success = True
        return rexplan_resp


    def generate_explanation(self):

        explanation = "\\rspd=80\\"

        if self.num_turns == 0:

            explanation += "Eu mexi a bóla " + str(self.ball) + " para " + str(
                pt_action_dic[get_action_name(self.action)]) \
                           + " \\pau=50\\ o que me dá " + str(self.score) + " pontos \\pau=700\\ Mas se tivesse mexido a bóla \\pau=50\\ " \
                           + str(self.best_ball) + " \\pau=50\\ para " + str(pt_action_dic[get_action_name(self.best_action)]) \
                           + " \\pau=50\\ podia conseguir " + str(self.best_score) + " pontos\\pau=700\\. Agora é a tua vez."


        elif self.num_turns == 1:

            explanation += "Eu mexi a bóla " + str(self.ball) + " para " + str(pt_action_dic[get_action_name(self.action)])\
                           + " \\pau=50\\  o que me poderia dar " + str(self.score) + " pontos no proximo turno \\pau=700\\ Mas\\pau=50\\  se tivesse mexido a bóla "\
                           + str(self.best_ball) + " para " + str(pt_action_dic[get_action_name(self.best_action)])\
                           + " \\pau=50\\ poderia conseguir " + str(self.best_score) + " pontos no proximo turno \\pau=700\\. Agora é a tua vez."
        else:

            explanation += "Eu mexi a bóla " + str(self.ball) + " para " + str(pt_action_dic[get_action_name(self.action)]) \
                           + " o que me poderia \\pau=50\\ dar " + str(self.score) + "\\pau=50\\  pontos daqui a " + str(self.num_turns) + " turnos \\pau=700\\ Mas\\pau=50\\  se tivesse mexido a bóla " \
                           + str(self.best_ball) + " \\pau=50\\ para " + str(pt_action_dic[get_action_name(self.best_action)]) \
                           + " \\pau=50\\ poderia conseguir " + str(self.best_score) + " \\pau=50\\ pontos daqui a " + str(self.num_turns) + " turnos \\pau=700\\. Agora é a tua vez."


        return explanation





    def clean_explanation_variables(self):
        self.depth = None
        self.ball = None
        self.action = None
        self.score = None
        self.best_ball = None
        self.best_action = None
        self.best_score = None
        self.num_turns = None
        self.explanation_text = None
        return


    def wake_robot(self):
        # Turn on Leds
        self.setup_leds(turn_on=True)

        # Turn on Face Tracking
        self.setup_face_tracking(0.1, turn_on=True)



    def setup_face_tracking(self, faceSize, turn_on=True):
        """ Robot starts to track the users face
        """
        if turn_on:

            # First, wake up
            self.motionProxy.wakeUp()
            self.motionProxy.rest()

            # Introduction
            self.postureProxy.goToPosture("Crouch", 1.0)
            self.robot_communication.say(
                 "Olá, eu sou o Nao. E tu? Como te chamas? \
                                              \\pau=1500\\ \
                                              Prazer conhecer-te.\
                                              \\pau=600\\ \
                                              Conheces um jogo chamado Guerra da minicalculadôra? \
                                              \\pau=1500\\ \
                                              Nao faz mal, vamos aprender a jogar juntos! \
                                              \\pau=600\\ \
                                              Preparado? Vou explicar-te como se joga. \
                                              \\pau=600\\ \
                                              O jogo usa a mini calculadora de pá \\pau=30\\pi \
                                              \\pau=600\\ \
                                              Ha dois jogadores: tú\\pau=80\\ e \\pau=80\\eu, e vamos jogar à vez \
                                              \\pau=600\\ \
                                              Como podes ver no ecra, um jogador começa com 1000 pontos, o outro jogador começa com 5.\
                                              \\pau=600\\ \
                                              So podes mexer uma bóla de cada vez\\pau=80\\ e\\pau=80\\cada quadrado so pode ter uma bóla\\pau=180\\. Não pódes sáltar quadrados. \
                                              \\pau=1000\\ \
                                              Quem jogar com as bólas azuís, tem que conseguir ficar com menos\\pau=50\\ pontos que o adversário.\
                                              Se conseguír, ganha o jogo.\
                                              \\pau=600\\ \
                                              Quem jogar com as bólas vermelhas, tem que conseguir ficar com mais pontos que o adversário.  \
                                              Se conseguír os mesmos pontos, ou mais, ganha. \
                                              \\pau=600\\ \
                                              Pelo menos foi assim que eu percebi as rregras. Percebêste? \
                                              \\pau=1000\\ \
                                              Boa! Vamos jogar!")



            #self.robot_communication.say("Adeus Nuno, muito obrigado")

            # Add target to track
            targetName = "Face"
            faceWidth = faceSize
            self.tracker.registerTarget(targetName, faceWidth)

            # Then, start tracker
            self.tracker.track(targetName)

        else:
            self.tracker.stopTracker()
            self.tracker.unregisterAllTargets()


    def setup_leds(self, turn_on=True):

        if turn_on:
            section1 = ["FaceLeds", "ChestLeds"]
            self.ledProxy.createGroup("turn", section1)
            self.ledProxy.fadeRGB("turn", 0x00FFFFFF, 0.3)
        else:
            section1 = ["FaceLeds", "ChestLeds"]
            self.ledProxy.createGroup("turn", section1)
            self.ledProxy.fadeRGB("turn", 0x00000000, 0.3)



if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    rospy.loginfo("[Robot Manager] Node is running...")
    rm = RobotManager()
    rospy.spin()















