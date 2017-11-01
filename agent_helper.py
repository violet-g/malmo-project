import time
import json
import numpy as np
import math

from state_space import StateSpace
from init_mission import init_mission

import MalmoPython

# This class implements a helper Agent for deriving the state-space representation ---#
class AgentHelper:
    """ This agent determines the state space for use by the actual problem solving agent. Enabeling do_plot will allow you to visualize the results """

    def __init__(self, agent_host, agent_port, mission_type, mission_seed, solution_report, state_space):
        """ Constructor for the helper agent """
        self.AGENT_NAME = 'Helper'
        self.AGENT_MOVEMENT_TYPE = 'Absolute' # Note the helper needs absolute movements
        self.DO_PLOT = False

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = StateSpace()
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    def run_agent(self):
        """ Run the Helper agent to get the state-space """

        #-- Load and init the Helper mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)

        #-- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        time.sleep(1)

        #-- Get the state of the world along with internal agent state...--#
        state_t = self.agent_host.getWorldState()

        #-- Get a state-space model by observing the Orcale/GridObserver--#
        if state_t.is_mission_running:
            #-- Make sure we look in the right direction when observing the surrounding (otherwise the coordinate system will rotated by the Yaw !) --#
            # Look East (towards +x (east) and +z (south) on the right, i.e. a std x,y coordinate system) yaw=-90
            self.agent_host.sendCommand("setPitch 20")
            time.sleep(1)
            self.agent_host.sendCommand("setYaw -90")
            time.sleep(1)

            #-- Basic map --#
            state_t = self.agent_host.getWorldState()

            if state_t.number_of_observations_since_last_state > 0:
                msg = state_t.observations[-1].text                 # Get the details for the last observed state
                oracle_and_internal = json.loads(msg)               # Parse the Oracle JSON
                grid = oracle_and_internal.get(u'grid', 0)
                xpos = oracle_and_internal.get(u'XPos', 0)
                zpos = oracle_and_internal.get(u'ZPos', 0)
                ypos = oracle_and_internal.get(u'YPos', 0)
                yaw  = oracle_and_internal.get(u'Yaw', 0)
                pitch = oracle_and_internal.get(u'Pitch', 0)

                #-- Parste the JOSN string, Note there are better ways of doing this! --#
                full_state_map_raw = str(grid)
                full_state_map_raw=full_state_map_raw.replace("[","")
                full_state_map_raw=full_state_map_raw.replace("]","")
                full_state_map_raw=full_state_map_raw.replace("u'","")
                full_state_map_raw=full_state_map_raw.replace("'","")
                full_state_map_raw=full_state_map_raw.replace(" ","")
                aa=full_state_map_raw.split(",")
                vocs = list(set(aa))
                for word in vocs:
                    for i in range(0,len(aa)):
                        if aa[i]==word :
                            aa[i] = vocs.index(word)

                X = np.asarray(aa);
                nn = int(math.sqrt(X.size))
                X = np.reshape(X, [nn,nn]) # Note: this matrix/table is index as z,x

                #-- Visualize the discrete state-space --#
                if self.DO_PLOT:
                    print(yaw)
                    plt.figure(1)
                    imgplot = plt.imshow(X.astype('float'),interpolation='none')
                    plt.show()

                #-- Define the unique states available --#
                state_wall = vocs.index("stained_hardened_clay")
                state_impossible = vocs.index("stone")
                state_initial = vocs.index("emerald_block")
                state_goal = vocs.index("redstone_block")

                #-- Extract state-space --#
                offset_x = 100-math.floor(xpos);
                offset_z = 100-math.floor(zpos);

                state_space_locations = {}; # create a dict

                for i_z in range(0,len(X)):
                    for j_x in range(0,len(X)):
                        if X[i_z,j_x] != state_impossible and X[i_z,j_x] != state_wall:
                            state_id = "S_"+str(int(j_x - offset_x))+"_"+str(int(i_z - offset_z) )
                            state_space_locations[state_id] = (int(j_x- offset_x),int(i_z - offset_z) )
                            if X[i_z,j_x] == state_initial:
                                state_initial_id = state_id
                                loc_start = state_space_locations[state_id]
                            elif X[i_z,j_x] == state_goal:
                                state_goal_id = state_id
                                loc_goal = state_space_locations[state_id]

                #-- Generate state / action list --#
                # First define the set of actions in the defined coordinate system
                actions = {"west": [-1,0],"east": [+1,0],"north": [0,-1], "south": [0,+1]}
                state_space_actions = {}
                for state_id in state_space_locations:
                    possible_states = {}
                    for action in actions:
                        #-- Check if a specific action is possible --#
                        delta = actions.get(action)
                        state_loc = state_space_locations.get(state_id)
                        state_loc_post_action = [state_loc[0]+delta[0],state_loc[1]+delta[1]]

                        #-- Check if the new possible state is in the state_space, i.e., is accessible --#
                        state_id_post_action = "S_"+str(state_loc_post_action[0])+"_"+str(state_loc_post_action[1])
                        if state_space_locations.get(state_id_post_action) != None:
                            possible_states[state_id_post_action] = 1

                    #-- Add the possible actions for this state to the global dict --#
                    state_space_actions[state_id] = possible_states

                #-- Kill the agent/mission --#
                self.agent_host.sendCommand("tp " + str(0 ) + " " + str(0) + " " + str(0))
                time.sleep(2)

                #-- Save the info an instance of the StateSpace class --
                self.state_space.state_actions = state_space_actions
                self.state_space.state_locations = state_space_locations
                self.state_space.start_id = state_initial_id
                self.state_space.start_loc = loc_start
                self.state_space.goal_id  = state_goal_id
                self.state_space.goal_loc = loc_goal

            #-- Reward location and values --#
            # OPTIONAL: If you want to account for the intermediate rewards
            # in the Random/Simple agent (or in your analysis) you can
            # obtain ground-truth by teleporting with the tp command
            # to all states and detect whether you recieve recieve a
            # diamond or not using the inventory field in the oracle variable
            #
            # As default the state_space_rewards is just set to contain
            # the goal state which is found above.
            #
            state_space_rewards = {}
            state_space_rewards[state_goal_id] = reward_goal

            # HINT: You can insert your own code for getting
            # the location of the intermediate rewards
            # and populate the state_space_rewards dict
            # with more information (optional).
            # WARNING: This is a bit tricky, please consult tutors before starting

            #-- Set the values in the state_space container --#
            self.state_space.reward_states = state_space_rewards
            self.state_space.reward_states_n = n_intermediate_rewards + 1
            self.state_space.reward_timeout = reward_timeout
            self.state_space.timeout = timeout
            self.state_space.reward_sendcommand = reward_sendcommand
        else:
            self.state_space = None
            #-- End if observations --#

        return
