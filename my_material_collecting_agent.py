from material_collecting_agent import MaterialCollectingAgent
from material_collecting_agent import MaterialCollectingAgentParameters
import random

class MyMaterialCollectingAgent(MaterialCollectingAgent):

    def __init__(self) -> None:
        super().__init__()
        pass

    def act(self):
        """
        A function that determines the action of the agent, sets its parameters accordingly, and updates the communication message.
        """
        # determine the action of the agent
        action = random.randint(0, MaterialCollectingAgentParameters.NUM_ACT-2)
        self.params.action = action
        # set the velocity and angular velocity based on the action
        if action == MaterialCollectingAgentParameters.ACT_GO_FORWARD:
            velocity = MaterialCollectingAgentParameters.MAX_VELOCITY
            self.params.velocity = velocity
        elif action == MaterialCollectingAgentParameters.ACT_ROTATE:
            ang_vel = random.random() * MaterialCollectingAgentParameters.MAX_ANGULAR_VELOCITY
            self.params.angular_velocity = ang_vel
        elif action == MaterialCollectingAgentParameters.ACT_STANDSTILL:
            pass
        # create a communication message based on the action
        c_message = "I'm ID" + str(self.id) + ", "
        if action == MaterialCollectingAgentParameters.ACT_GO_FORWARD:
            c_message += "I'm going forward with velocity " + str(velocity)
        elif action == MaterialCollectingAgentParameters.ACT_ROTATE:
            c_message += "I'm rotating with angular velocity " + str(ang_vel)
        elif action == MaterialCollectingAgentParameters.ACT_STANDSTILL:
            c_message += "I'm standing still"
        # set the communication message
        self.params.communication_message = c_message
        # read the messages received from other agents
        if len(self.params.received_messages) > 0:
            pass

