import time
import random
import json
import networkx as nx
import matplotlib.pyplot as plt

from agent_helper import AgentHelper
from init_mission import init_mission

import MalmoPython

# This class implements the Simple Agent --#
class AgentSimple:

    def __init__(self, agent_host, agent_port, mission_type, mission_seed, solution_report, state_space):
        """ Constructor for the simple agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Simple'

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = state_space;
        self.solution_report = solution_report;  # Python calls by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    def best_first_graph_search(problem, f):
        """Search the nodes with the lowest f scores first.
        You specify the function f(node) that you want to minimize; for example,
        if f is a heuristic estimate to the goal, then we have greedy best
        first search; if f is node.depth then we have breadth-first search.
        There is a subtlety: the line "f = memoize(f, 'f')" means that the f
        values will be cached on the nodes as they are computed. So after doing
        a best first search you can examine the f values of the path returned."""

        # we use these two variables at the time of visualisations
        iterations = 0
        all_node_colors = []
        node_colors = dict(initial_node_colors)

        f = memoize(f, 'f')
        node = Node(problem.initial)

        node_colors[node.state] = "red"
        iterations += 1
        all_node_colors.append(dict(node_colors))

        if problem.goal_test(node.state):
            node_colors[node.state] = "green"
            iterations += 1
            all_node_colors.append(dict(node_colors))
            return(iterations, all_node_colors, node)

        frontier = PriorityQueue(min, f)
        frontier.append(node)

        node_colors[node.state] = "orange"
        iterations += 1
        all_node_colors.append(dict(node_colors))

        explored = set()
        while frontier:
            node = frontier.pop()

            node_colors[node.state] = "red"
            iterations += 1
            all_node_colors.append(dict(node_colors))

            if problem.goal_test(node.state):
                node_colors[node.state] = "green"
                iterations += 1
                all_node_colors.append(dict(node_colors))
                return(iterations, all_node_colors, node)

            explored.add(node.state)
            for child in node.expand(problem):
                if child.state not in explored and child not in frontier:
                    frontier.append(child)
                    node_colors[child.state] = "orange"
                    iterations += 1
                    all_node_colors.append(dict(node_colors))
                elif child in frontier:
                    incumbent = frontier[child]
                    if f(child) < f(incumbent):
                        del frontier[incumbent]
                        frontier.append(child)
                        node_colors[child.state] = "orange"
                        iterations += 1
                        all_node_colors.append(dict(node_colors))

            node_colors[node.state] = "gray"
            iterations += 1
            all_node_colors.append(dict(node_colors))
        return None

    def astar_search(problem, h=None):
        """A* search is best-first graph search with f(n) = g(n)+h(n).
        You need to specify the h function when you call astar_search, or
        else in your Problem subclass."""
        h = memoize(h or problem.h, 'h')
        iterations, all_node_colors, node = best_first_graph_search(problem, lambda n: n.path_cost + h(n))
        return(iterations, all_node_colors, node)

    def run_agent(self):
        # Need this to import GraphProblem from AIMA
        from search import *

        """ Run the Simple agent and log the performance and resource use """

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        # INSERT: YOUR SOLUTION HERE (REMEMBER TO MANUALLY UPDATE THE solution_report DEPENDING ON YOUR SOLUTION)
        state_space = self.state_space
        print(state_space.goal_id)
        print(state_space.goal_loc)

        maze_map = UndirectedGraph(state_space.state_actions)
        maze_map_locations = state_space.state_locations
        print(state_space.state_actions)
        print(maze_map)

		# initialise a graph
        G = nx.Graph()

		# use this while labeling nodes in the map
        node_labels = dict()
        node_colors = dict()
        for n, p in maze_map_locations.items():
            G.add_node(n)            # add nodes from locations
            node_labels[n] = n       # add nodes to node_labels
            node_colors[n] = "white" # node_colors to color nodes while exploring the map

		# we'll save the initial node colors to a dict for later use
        initial_node_colors = dict(node_colors)
    
        
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
        
        print("Done creating the graph object")

    
        # Create the maze_problem using AIMA 
        maze_problem = GraphProblem(state_space.start_id, state_space.goal_id, maze_map)
        print("Initial state:"+state_space.start_id) # change to get actual goal
        print("Goal state:"+state_space.goal_id)

        all_node_colors=[]
        iterations, all_node_colors, node = astar_search(problem=maze_problem, h=None)

		#-- Trace the solution --#
        solution_path = [node]
        cnode = node.parent
        solution_path.append(cnode)
        while cnode.state != "S_0_0":    
            cnode = cnode.parent  
            solution_path.append(cnode)

        print("----------------------------------------")
        print("Identified goal state:"+str(solution_path[0]))
        print("----------------------------------------")
        print("Solution trace:"+str(solution_path))
        print("----------------------------------------")
        print("Final solution path:")
        show_map(final_path_colors(maze_problem, node.solution()))

        solution_path_local = deepcopy(solution_path)
        print(solution_path_local)

        state_t = self.agent_host.getWorldState()

        # Main loop:
        while state_t.is_mission_running:

            target_node = solution_path_local.pop()
            try:                
                print("Action_t: Goto state " + target_node.state)
                if target_node.state == state_space.goal_id:
                    # Hack for AbsolutMovements: Do not take the full step to 1,9 ; then you will "die" we just need to be close enough (0.25)
                    x_new = 1
                    z_new = 8.75
                else:
                    xz_new = maze_map.locations.get(target_node.state);
                    x_new = xz_new[0] + 0.5 
                    z_new = xz_new[1] + 0.5 
                                    
                agent_host.sendCommand("tp " + str(x_new ) + " " + str(217) + " " + str(z_new))                 
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
                grid = oracle.get(u'grid', 0)           #

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)            # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)            # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)            # Height as measured from surface! (yes Y!)

                # Standard "internal" sensory inputs
                yaw  = oracle.get(u'Yaw', 0)             #
                pitch = oracle.get(u'Pitch', 0)          #

            # Vision
            #if state_t.number_of_video_frames_since_last_state > 0: # Have any Vision percepts been registred ?
            #   frame = state_t.video_frames[0]

            #-- Print some of the state information --#
            print("Percept: video,observations,rewards received:",state_t.number_of_video_frames_since_last_state,state_t.number_of_observations_since_last_state,state_t.number_of_rewards_since_last_state)
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(xpos) + " " + str(ypos) + " " + str(zpos)+ " " + str(yaw) + " " + str(pitch))

        # --------------------------------------------------------------------------------------------
        # Summary
        print("Summary:")
        print("Mission has ended ... either because time has passed (-1000 reward) or goal reached (1000 reward) or early stop (0 reward)")
        print("Cumulative reward = " + str(reward_cumulative) )
        return
