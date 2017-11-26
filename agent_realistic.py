import time
import datetime
import random
import json
import numpy as np
import pickle

from agent_helper import AgentHelper
from init_mission import init_mission

import MalmoPython

#-- this class implements agent Realistic
class AgentRealistic:

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the realistic agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete'
        self.AGENT_NAME = 'Realistic'
        # define the allowed actions
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movewest 1", "moveeast 1", "movesouth 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = None; # The Realistic agent knows nothing about the state_space a priori
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
        # add action to solution report
        self.solution_report.addAction()
        return actual_action

    def run_agent(self):
        """ Run the Realistic agent and log the performance and resource use """

        # open Q table file
        try:
            Q = pickle.load(open("Q.pickle", "rb"))
        except (OSError, IOError) as e:
            # if the Q table does not exist, create it
            print "Creating Q ..."
            # Q table is fixed sized - for mission of size "small"
            Q = [[100] * 4 for i in range(100)] # 100 states in a maze 10x10; 4 actions - movenorth, movewest, moveeast, movesouth

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        #-- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        BLOCK_TYPES = {"stone":-1000, "glowstone":0, "emerald_block":0, "stained_hardened_clay": -1000, "redstone_block": 1000}
        alpha = 0.9 # learning factor
        gamma = 0.8 # greediness of the agent
        epsilon = 0.1 # controlled noise - take random action in 10% of the times
        noise = 0.0 # this noise is not used, substituted by epsilon
        reward_cumulative = 0.0

        # get curret state
        state_t = self.agent_host.getWorldState()

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

                x = int(xpos) # round down
                z = int(zpos)    # e.g. 8.5 -> 8
                curr_s = 10*z + x # the current state of the agent as index in the Q table
                print "\nCurrent position: " + str(x) + " " + str(z)

                # learn about all walls surrounding the agent
                for i in range(len(grid)):
                    if i%2 != 0 and BLOCK_TYPES[str(grid[i])] < 0:
                        # populate table with -100 for a wall
                        Q[curr_s][(i-1)/2] = -100

                # generate a random number
                rnd = random.random()
                curr_a_idx = None
                allowed_actions_idx = [] # array stores ints between 0 and 3 -> correspond to indices in AGENT_ALLOWED_ACTIONS
                if rnd < epsilon: # take random action
                    for i in range(len(grid)):
                        # if the action is north, south, west, east (disregard diagonals) and is not a wall
                        if i%2 != 0 and BLOCK_TYPES[str(grid[i])] >= 0:
                            allowed_actions_idx.append((i-1)/2) # map index from grid (3x3) to array of size 4
                else: # choose action based on Q table values
                    max_val = max(Q[curr_s]) # find largest Q value
                    for i in range(len(Q[curr_s])):
                        # find all entries with the max value and make sure they are not a wall
                        if Q[curr_s][i] == max_val and BLOCK_TYPES[str(grid[i*2 + 1])] >= 0:
                            allowed_actions_idx.append(i)

                # choose an action from the allowed (either random or from Q table depending on epsilon above)
                curr_a_idx = random.choice(allowed_actions_idx)

                # check if this state is the goal state
                is_final_goal = False
                if BLOCK_TYPES[str(grid[curr_a_idx * 2 + 1])] == 1000:
                    is_final_goal = True

                # execute action, this noise is 0, as action is already randomised
                curr_a = self.__ExecuteActionForRealisticAgentWithNoisyTransitionModel__(curr_a_idx, noise)

                # get rewards
                r = sum(r.getValue() for r in state_t.rewards)
                if (r < 0): # do not take negative rewards
                    r = 0
                if is_final_goal: # get the reward for the final state - cannot be taken otherwise as the agent cannot go inside the wall
                    r += 1000.0
                reward_cumulative += r
                self.solution_report.addReward(r, datetime.datetime.now())
                print("Reward:", r)
                print("Cummulative reward so far:", reward_cumulative)

                # calculate next state based on the chosen action
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

                # update Q table
                Q[curr_s][curr_a_idx] = (1-alpha)*Q[curr_s][curr_a_idx] +\
                    alpha*(r + gamma*(max(Q[next_s])))

            # get new state and sleep to make sure observations are gathered
            state_t = self.agent_host.getWorldState()
            time.sleep(0.5)

        # save Q table updates
        pickle.dump(Q, open("Q.pickle", "wb"))

        self.solution_report.checkGoal()
        
        return
