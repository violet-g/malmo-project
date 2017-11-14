import time
import random
import json
import networkx as nx
import matplotlib.pyplot as plt

from agent_helper import AgentHelper
from init_mission import init_mission
from copy import deepcopy
import os
import sys

import MalmoPython

AIMA_TOOLBOX_ROOT=os.environ['AIMA_PATH']
sys.path.append(AIMA_TOOLBOX_ROOT)
from search import *

# This class implements the Simple Agent
class AgentSimple:

    def __init__(self, agent_host, agent_port, mission_type, mission_seed, solution_report, state_space):
        """ Constructor for the simple agent """
        self.AGENT_MOVEMENT_TYPE = 'Absolute' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Simple'

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = state_space
        self.solution_report = solution_report  # Python calls by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    def run_agent(self):
        """ Run the Simple agent and log the performance and resource use """

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        # INSERT: YOUR SOLUTION HERE (REMEMBER TO MANUALLY UPDATE THE solution_report DEPENDING ON YOUR SOLUTION)
        state_space = self.state_space
        agent_host = self.agent_host

        maze_map = UndirectedGraph(state_space.state_actions)
        maze_map.locations = state_space.state_locations
        maze_map_locations = maze_map.locations

		# initialise a graph
        G = nx.Graph()

		# use this while labeling nodes in the map
        node_labels = dict()
        for n, p in maze_map_locations.items():
            G.add_node(n)            # add nodes from locations
            node_labels[n] = n       # add nodes to node_labels

		# positions for node labels
        node_label_pos = {k:[v[0],v[1]-0.25] for k,v in maze_map_locations.items()} # spec the position of the labels relative to the nodes

		# use this while labeling edges
        edge_labels = dict()

		# add edges between nodes in the map - UndirectedGraph defined in search.py
        for node in maze_map.nodes():
            connections = maze_map.get(node)
            for connection in connections.keys():
                distance = connections[connection]
                G.add_edge(node, connection) # add edges to the graph
                edge_labels[(node, connection)] = distance # add distances to edge_labels

        # Create the maze_problem using AIMA
        maze_problem = GraphProblem(state_space.start_id, state_space.goal_id, maze_map)
        print("Initial state:"+maze_problem.initial)
        print("Goal state:"+maze_problem.goal)

        node = astar_search(problem=maze_problem, h=None)

		#-- Trace the solution --#
        solution_path = [node]
        cnode = node.parent
        solution_path.append(cnode)
        while cnode.state != maze_problem.initial:
            cnode = cnode.parent
            solution_path.append(cnode)

        print("----------------------------------------")
        print("Identified goal state:"+str(solution_path[0]))
        print("----------------------------------------")
        print("Solution trace:"+str(solution_path))
        print("----------------------------------------")

        solution_path_local = deepcopy(solution_path)

        state_t = agent_host.getWorldState()

        agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)

        reward_cumulative = 0.0

        # Main loop:
        while state_t.is_mission_running:

            target_node = solution_path_local.pop()
            try:
                print("Action_t: Goto state " + target_node.state)
                if target_node.state == state_space.goal_id:
                    # Hack for AbsolutMovements: Do not take the full step to 1,9 ; then you will "die" we just need to be close enough (0.25)
                    xz_new = maze_map.locations.get(target_node.state)
                    x_new = xz_new[0]
                    z_new = xz_new[1] - 0.25
                else:          

                    xz_new = maze_map.locations.get(target_node.state)
                    x_new = xz_new[0] + 0.5
                    z_new = xz_new[1] + 0.5

                agent_host.sendCommand("tp " + str(x_new) + " " + str(217) + " " + str(z_new))
            except RuntimeError as e:
                print "Failed to send command:",e
                pass

            # Wait 0.5 sec
            time.sleep(0.5)

            # Get the world state

            state_t = agent_host.getWorldState() # might need .self here

            # Collect the number of rewards and add to reward_cumulative
            # Note: Since we only observe the sensors and environment every a number of rewards may have accumulated in the buffer
            for reward_t in state_t.rewards:
                print("Reward_t:",reward_t.getValue())
                reward_cumulative += reward_t.getValue()

            # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:",error.text)

            # Handle the sensor input
            xpos = None
            ypos = None
            zpos = None
            yaw  = None
            pitch = None
            if state_t.number_of_observations_since_last_state > 0: # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text      # Get the detailed for the last observed state
                oracle = json.loads(msg)                 # Parse the Oracle JSON

                # Oracle
                grid = oracle.get(u'grid', 0)

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)            # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)            # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)            # Height as measured from surface! (yes Y!)

                # Standard "internal" sensory inputs
                yaw  = oracle.get(u'Yaw', 0)             #
                pitch = oracle.get(u'Pitch', 0)          #

            # Vision
            if state_t.number_of_video_frames_since_last_state > 0: # Have any Vision percepts been registred ?
               frame = state_t.video_frames[0]

            #-- Print some of the state information --#
            print("Percept: video,observations,rewards received:",state_t.number_of_video_frames_since_last_state,state_t.number_of_observations_since_last_state,state_t.number_of_rewards_since_last_state)
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(xpos) + " " + str(ypos) + " " + str(zpos)+ " " + str(yaw) + " " + str(pitch))

        # --------------------------------------------------------------------------------------------
        # Summary
        print("Summary:")
        print("Mission has ended ... either because time has passed (-1000 reward) or goal reached (1000 reward) or early stop (0 reward)")
        print("Cumulative reward = " + str(reward_cumulative) )
        return
