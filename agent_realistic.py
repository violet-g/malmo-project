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
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = None; # NOTE: The Realistic can not know anything about the state_space a priori !
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    #----------------------------------------------------------------------------------------------------------------#
    def __ExecuteActionForRealisticAgentWithNoisyTransitionModel__(idx_requested_action, noise_level):
        """ Creates a well-defined transition model with a certain noise level """
        n = len(self.AGENT_ALLOWED_ACTIONS)
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_request_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distribution of actions
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]
        self.agent_host.sendCommand(actual_action)
        return actual_action

    #----------------------------------------------------------------------------------------------------------------#
    def run_agent(self):
        """ Run the Realistic agent and log the performance and resource use """

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        BLOCK_TYPES = {"stone":-1, "glowstone":0, "emerald_block":0, "stained_hardened_clay": -1}

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

        if state_t.number_of_observations_since_last_state > 0: # Has any Oracle-like and/or internal sensor observations come in?
            msg = state_t.observations[-1].text      # Get the detailed for the last observed state
            oracle = json.loads(msg)                 # Parse the Oracle JSON

            # Oracle
            grid = oracle.get(u'grid', 0)
            print grid

            # GPS-like sensor
            xpos = oracle.get(u'XPos', 0)            # Position in 2D plane, 1st axis
            zpos = oracle.get(u'ZPos', 0)            # Position in 2D plane, 2nd axis (yes Z!)
            ypos = oracle.get(u'YPos', 0)


            
        q_table = np.matrix([[0,0,0],[0,0,0],[0,0,0]])
        reward_matrix = np.matrix([[0,0,0],[0,0,0],[0,0,0]])
        gamma = 0                                   # Greediness of the agent. Closer to 0 is greedier
        i = 0

        for block in grid:
            reward_matrix.put(i, BLOCK_TYPES[str(block)])
            i += 1


        print reward_matrix

        #get state_t 
        state_t = self.agent_host.getWorldState()

		#Main Loop
        while state_t.is_mission_running: 

            # maybe TODO: do while the goal state hasn't been reached   

			#Set the world state
            state_t = self.agent_host.getWorldState()

            # Look at current frame
            if state_t.number_of_video_frames_since_last_state > 0: # Have any Vision percepts been registred ?
               frame = state_t.video_frames[0]



			# Stop movement
            if state_t.is_mission_running:
                # Enforce a simple discrete behavior by stopping any continuous movement in progress

                if continuousMovement:
                    self.agent_host.sendCommand("move "  + str(0))
                    self.agent_host.sendCommand("pitch " + str(0))
                    self.agent_host.sendCommand("turn "  + str(0))
                else:
                    actionIdx = random.randint(0, 2)
                    self.agent_host.sendCommand(discreteAction[actionIdx])

			
        return
