from init_mission import init_mission
import time

from agent_helper import AgentHelper

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

    def run_agent(self):
        """ Run the Simple agent and log the performance and resource use """

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        # INSERT: YOUR SOLUTION HERE (REMEMBER TO MANUALLY UPDATE THE solution_report DEPENDING ON YOU SOLUTION)

        return
