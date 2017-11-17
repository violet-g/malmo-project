import time
import random
import json
import numpy as np

from agent_helper import AgentHelper
from init_mission import init_mission

import MalmoPython

# This class implements the Realistic Agent --#
class AgentRealistic:

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the realistic agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Realistic'
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "moveeast 1", "movewest 1",  "movesouth 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = None; # NOTE: The Realistic can not know anything about the state_space a priori !
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    #----------------------------------------------------------------------------------------------------------------#
    def __ExecuteActionForRealisticAgentWithNoisyTransitionModel__(self, idx_requested_action, noise_level):
        """ Creates a well-defined transition model with a certain noise level """
        n = len(self.AGENT_ALLOWED_ACTIONS)
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_requested_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distribution of actions
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]
        print(actual_action)
        self.agent_host.sendCommand(actual_action)
        return actual_action

    #----------------------------------------------------------------------------------------------------------------#
    def run_agent(self, q_table):
        """ Run the Realistic agent and log the performance and resource use """

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        BLOCK_TYPES = {"stone":-1000, "glowstone":0, "emerald_block":0, "stained_hardened_clay": -1000}

        # INSERT YOUR SOLUTION HERE (REWARDS MUST BE UPDATED IN THE solution_report)
        #
        # NOTICE: YOUR FINAL AGENT MUST MAKE USE OF THE FOLLOWING NOISY TRANSISION MODEL
        #       ExecuteActionForRealisticAgentWithNoisyTransitionModel(idx_requested_action, 0.05)
        #   FOR DEVELOPMENT IT IS RECOMMENDED TO FIST USE A NOISE FREE VERSION, i.e.
        #       ExecuteActionForRealisticAgentWithNoisyTransitionModel(idx_requested_action, 0.0)

        continuousMovement = False

        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.SUM_REWARDS)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)

        # Goal:
        # goal_t: The goal is obtained when the cumulative reward reaches 1000 (checked internally in the mission definition)
        # Let's predefine the cumulative reward - note the goal test is (effectively) checked against this value
        reward_cumulative = 0.0

        state_t = self.agent_host.getWorldState()
        print state_t, state_t.number_of_observations_since_last_state

        if state_t.number_of_observations_since_last_state > 0: # Has any Oracle-like and/or internal sensor observations come in?
            frame = state_t.video_frames[0]
            msg = state_t.observations[-1].text      # Get the detailed for the last observed state
            oracle = json.loads(msg)                 # Parse the Oracle JSON

            # Oracle
            grid = oracle.get(u'grid', 0)

            # GPS-like sensor
            xpos = oracle.get(u'XPos', 0)            # Position in 2D plane, 1st axis
            zpos = oracle.get(u'ZPos', 0)            # Position in 2D plane, 2nd axis (yes Z!)
            ypos = oracle.get(u'YPos', 0)

        reward_matrix = [[0]*3,[0]*3,[0]*3]
        gamma = 1                                   # Greediness of the agent. Closer to 0 is greedier
        alpha = 1                                   # What is alpha?

        prev_s = None
        prev_a = None
        prev_r = 0

        x_curr = int(xpos) # int rounds xpos down from 8.5 to 8
        z_curr = int(zpos)
        curr_s = (zpos*10)+xpos+1
        curr_a = None
        curr_r = 0
        noise = 0 # TODO: change this
        print "Initial position: " + str(x_curr) + " " + str(z_curr)

        for i in range(len(grid)):
            reward_matrix[(i-i%3)/3][i%3] = BLOCK_TYPES[str(grid[i])]

        print (reward_matrix)

        #  Take the first action here
        # Create a reward table based on block types
        for row in reward_matrix:
            for i in range(len(row)):
                if reward_matrix.index(row) == 0 and i == 1:
                    q_table[(z_curr * 10) + x_curr + 1][reward_matrix.index(row)] = row[i] # movenorth
                elif reward_matrix.index(row) == 1:
                    if i == 0:
                        q_table[(z_curr * 10) + x_curr + 1][2] = row[i] # movewest
                    elif i == 2:
                        q_table[(z_curr * 10) + x_curr + 1][1] = row[i] # moveeast
                elif reward_matrix.index(row) == 2 and i == 1:
                    q_table[(z_curr * 10) + x_curr + 1][reward_matrix.index(row)] = row[i] # movesouth

        action_index = np.argmax(q_table[(z_curr * 10) + x_curr + 1])

        print q_table
        prev_s = (z_curr * 10) + x_curr + 1  # State between 0 and 100 to map onto the q_table
        prev_a = self.__ExecuteActionForRealisticAgentWithNoisyTransitionModel__(action_index, noise)
        time.sleep(0.5)
        x_old = x_curr
        z_old = z_curr

        #Main Loop
        while state_t.is_mission_running:

            state_t = self.agent_host.getWorldState()

            if state_t.number_of_observations_since_last_state > 0:  # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text  # Get the detailed for the last observed state
                oracle = json.loads(msg)  # Parse the Oracle JSON

                # Oracle
                grid = oracle.get(u'grid', 0)

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)  # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)  # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)

            x_curr = int(xpos)  # int rounds xpos down from 8.5 to 8
            z_curr = int(zpos)
            curr_s = (zpos * 10) + xpos + 1
            curr_a = None
            curr_r = 0
            print "Current position: " + str(x_curr) + " " + str(z_curr)

            for i in range(len(grid)):
                reward_matrix[(i - i % 3) / 3][i % 3] = BLOCK_TYPES[str(grid[i])]

            print (reward_matrix)

            #  Take an action
            # Create a reward table based on block types
            for row in reward_matrix:
                for i in range(len(row)):
                    old_q = q_table[prev_s][self.AGENT_ALLOWED_ACTIONS.index(prev_a)]

                    q_table[prev_s][self.AGENT_ALLOWED_ACTIONS.index(prev_a)] = \
                        old_q + alpha*(prev_r + gamma*(max(q_table[curr_s][curr_a]) - old_q))

            action_index = np.argmax(q_table[(z_curr * 10) + x_curr + 1])

            print q_table
            prev_s = (z_curr * 10) + x_curr + 1  # State between 0 and 100 to map onto the q_table
            prev_a = self.AGENT_ALLOWED_ACTIONS[action_index]
            x_old = x_curr
            z_old = z_curr
            self.__ExecuteActionForRealisticAgentWithNoisyTransitionModel__(action_index, noise)
            time.sleep(0.5)


            # Stop movement
            if state_t.is_mission_running:
                # Enforce a simple discrete behavior by stopping any continuous movement in progress

                if continuousMovement:
                    self.agent_host.sendCommand("move "  + str(0))
                    self.agent_host.sendCommand("pitch " + str(0))
                    self.agent_host.sendCommand("turn "  + str(0))
                else:
                    actionIdx = random.randint(0, 2)
                    # self.agent_host.sendCommand(discreteAction[actionIdx])

            
        return
