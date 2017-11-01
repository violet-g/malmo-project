import time
import random
import json

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
        """ Run the Simple agent and log the performance and resource use """

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        # INSERT: YOUR SOLUTION HERE (REMEMBER TO MANUALLY UPDATE THE solution_report DEPENDING ON YOU SOLUTION)
        state_space_locations = self.state_space.state_locations
        print(state_space_locations)
        
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        # Fix the randomness of the agent by seeding the random number generator
        random.seed(1)
        agentConfusionLevel = 0.1
        continuousMovement = False
        discreteAction = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]

        # Goal:
        # goal_t: The goal is obtained when the cumulative reward reaches 1000 (checked internally in the mission definition)
        # Let's predefine the cumulative reward - note the goal test is (effectively) checked against this value
        reward_cumulative = 0.0

        state_t = self.agent_host.getWorldState()

        # Main loop:
        while state_t.is_mission_running:

            # This is a senseless agent with random movement
            try:
                # Random hardwired moves for demo only
                self.agent_host.sendCommand("pitch " + str(0)) # 0: look straigh ahead
                self.agent_host.sendCommand("move "  + str(1)) # 1: means: move forward as fast as possible (in direction of sight)
                self.agent_host.sendCommand("turn "  + str(agentConfusionLevel*(random.random()*2-1)) ) # start turing in a random direction, rather slowly
            except RuntimeError as e:
                print("Failed to send command:",e)
                pass

            # Wait 0.5 sec
            time.sleep(0.5)

            # Set the world state
            state_t = self.agent_host.getWorldState()

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
