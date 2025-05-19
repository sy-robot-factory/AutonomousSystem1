# -*- coding: utf-8 -*-

from agent import Agent
import numpy

class MaterialCollectingAgentParameters():

    ACT_GO_FORWARD = 0
    ACT_ROTATE = 1
    ACT_STANDSTILL = 2
    
    NUM_ACT = 3

    MAX_VELOCITY = 100.0
    MAX_ANGULAR_VELOCITY = 100.0

    SENSE_NONE = 0
    SENSE_AGENT = 1
    SENSE_OBSTACLE = 2
    SENSE_MATERIAL = 3

    SENSE_COLLIDED = 1
    SENSE_NOT_COLLIDED = 0

    NUM_SENSOR = 5


    def __init__(self) -> None:
        ##1. if you want this agent to move forward, set action = ACT_GO_FORWARD and
        ##set velocity.
        ##2. if you want this agent to rotate, set action = ACT_ROTATE and set the rotation angle
        ## to rotation_angle
        ##3. if you want this agent to stop at its current position, set action = ACT_STANDSTILL
        ##
        self.action = MaterialCollectingAgentParameters.ACT_STANDSTILL
        self.velocity = 0
        self.angular_velocity = 0
        self.sensor_object_type = numpy.full(MaterialCollectingAgentParameters.NUM_SENSOR, MaterialCollectingAgentParameters.SENSE_NONE)
        self.sensor_object_distance = numpy.full(MaterialCollectingAgentParameters.NUM_SENSOR, 0.0)
        self.sensor_object_attribute = numpy.full(MaterialCollectingAgentParameters.NUM_SENSOR, 0.0)

        ## add collision sensor
        self.collision_sensor = MaterialCollectingAgentParameters.SENSE_NOT_COLLIDED

        #variables for communication
        self.communication_message = ""
        self.received_messages = []

        assert MaterialCollectingAgentParameters.NUM_SENSOR % 2 == 1 # must be odd


class MaterialCollectingAgent(Agent):

    def __init__(self) -> None:
        super().__init__()
        self.params = MaterialCollectingAgentParameters()

    def act(self):
        pass


    