import time
import datetime
import random
import json
import numpy as np
import pickle

from agent_helper import AgentHelper
from init_mission import init_mission

import MalmoPython

class AgentRealistic:

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the realistic agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Realistic'
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movewest 1", "moveeast 1", "movesouth 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = None; # NOTE: The Realistic can not know anything about the state_space a prior i !
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    def __ExecuteActionForRealisticAgentWithNoisyTransitionModel__(self, idx_request_action, noise_level):
        """ Creates a well-defined transition model with a certain noise level """
        """ Note: You can modify this funcitonif you need to  """
        n = len(self.AGENT_ALLOWED_ACTIONS)
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_request_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distrbution of actions
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]
        print(actual_action)
        self.agent_host.sendCommand(actual_action)
        return actual_action

    def run_agent(self):
        """ Run the Realistic agent and log the performance and resource use """

        try:
            Q = pickle.load(open("Q.pickle", "rb"))
        except (OSError, IOError) as e:
            print "Creating Q ..."
            Q = [[100] * 4 for i in range(100)]

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        # INSERT YOUR SOLUTION HERE (REWARDS MUST BE UPDATED IN THE solution_report)
        #
        # NOTICE: YOUR FINAL AGENT MUST MAKE USE OF THE FOLLOWING NOISY TRANSISION MODEL
        #       ExecuteActionForRealisticAgentWithNoisyTransitionModel(idx_requested_action, 0.05)
        #   FOR DEVELOPMENT IT IS RECOMMENDED TO FIST USE A NOISE FREE VERSION, i.e.
        #       ExecuteActionForRealisticAgentWithNoisyTransitionModel(idx_requested_action, 0.0)

        #-- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        BLOCK_TYPES = {"stone":-1000, "glowstone":0, "emerald_block":0, "stained_hardened_clay": -1000, "redstone_block": 1000}
        alpha = 0.9
        gamma = 0.8
        epsilon = 0.1
        noise = 0.0
        reward_cumulative = 0.0

        state_t = self.agent_host.getWorldState()

        #Main Loop
        while state_t.is_mission_running:
            if state_t.number_of_observations_since_last_state > 0:  # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text  # Get the detailed for the last observed state
                oracle = json.loads(msg)  # Parse the Oracle JSON

                # Oracle
                grid = oracle.get(u'grid', 0)

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)  # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)  # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)

                x = int(xpos)
                z = int(zpos)
                curr_s = 10*z + x
                print "\nCurrent position: " + str(x) + " " + str(z)

                for i in range(len(grid)):
                    if i%2 != 0 and BLOCK_TYPES[str(grid[i])] < 0:
                        Q[curr_s][(i-1)/2] = -100

                rnd = random.random()
                curr_a_idx = None
                allowed_actions_idx = [] # between 0 and 3
                if rnd < epsilon: # take random action
                    for i in range(len(grid)):
                        if i%2 != 0 and BLOCK_TYPES[str(grid[i])] >= 0:
                            allowed_actions_idx.append((i-1)/2)
                else: # choose action based on Q table values
                    max_val = max(Q[curr_s])
                    for i in range(len(Q[curr_s])):
                        if Q[curr_s][i] == max_val and BLOCK_TYPES[str(grid[i*2 + 1])] >= 0:
                            allowed_actions_idx.append(i)

                print allowed_actions_idx
                curr_a_idx = random.choice(allowed_actions_idx)
                curr_a = self.__ExecuteActionForRealisticAgentWithNoisyTransitionModel__(curr_a_idx, noise)

                r = sum(r.getValue() for r in state_t.rewards)
                if (r < 0):
                    r = 0
                reward_cumulative += r
                self.solution_report.addReward(r, datetime.datetime.now())
                print("Reward:", r)
                print("Cummulative reward so far:", reward_cumulative)

                next_x = x
                next_z = z
                if curr_a_idx == 0:
                    next_z -= 1
                elif curr_a_idx == 1:
                    next_x -= 1
                elif curr_a_idx == 2:
                    next_x += 1
                elif curr_a_idx == 3:
                    next_z += 1

                next_s = 10*next_z + next_x
                Q[curr_s][curr_a_idx] = (1-alpha)*Q[curr_s][curr_a_idx] +\
                    alpha*(r + gamma*(max(Q[next_s])))

            state_t = self.agent_host.getWorldState()
            time.sleep(0.2)

        pickle.dump(Q, open("Q.pickle", "wb"))
        return
