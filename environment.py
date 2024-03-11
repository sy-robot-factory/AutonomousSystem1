# -*- coding: utf-8 -*-

import pyglet


class Environment:

    def __init__(self) -> None:
        self.agents = []
        self.shapes = []
        self.agent_shapes = []
        self.agent_tags=[]
        self.object_shapes = []
        self.current_id = 0
        self.current_interval = 0.0
        self.agent_update_interval = 0.0
        self.agent_update_ready = False
        self.info_tags = []
        self.request_reset = False

        self.background_group = pyglet.graphics.OrderedGroup(0)
        self.main_group = pyglet.graphics.OrderedGroup(1)
        self.foreground_group = pyglet.graphics.OrderedGroup(2)


    def register_shapes(self, batch):
        pass

    def update(self, dt):
        """
        Update the current interval and check if the agent update is ready based on the time increment.
        :param dt: The time increment for the update.
        :return: None
        """
        self.current_interval += dt
        if self.current_interval >= self.agent_update_interval:
            self.agent_update_ready = True
            self.current_interval = 0.0
        else:
            self.agent_update_ready = False


    def register_agent(self, agent):
        """
        Register a new agent by assigning it a unique ID and adding it to the list of agents.
        Parameters:
            agent: the agent to be registered
        Return:
            None
        """
        agent.id = self.current_id
        self.current_id += 1
        self.agents.append(agent)

    def delete_shapes(self):
        pass
    


