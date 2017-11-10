"""
 Artificial Inteligence (H)
 Assessed Exercise 2017/2018

 Tested with Python 2.7

 Solution Template (revision a)
"""

from agent_random import AgentRandom
from agent_simple import AgentSimple
from agent_realistic import AgentRealistic
from agent_helper import AgentHelper
from state_space import StateSpace
from solution_report import SolutionReport

# The main entry point if you run the module as a script--#
if __name__ == "__main__":

    import os

    #-- Define default arguments, in case you run the module as a script --#
    DEFAULT_STUDENT_GUID = 'template'
    DEFAULT_AGENT_NAME   = 'Simple' #HINT: Currently choose between {Random,Simple, Realistic}
    DEFAULT_MALMO_PATH   = os.environ['MALMO_ROOT'] # HINT: Change this to your own path
    DEFAULT_AIMA_PATH    = os.environ['AIMA_PATH']  # HINT: Change this to your own path, forward slash only, should be the 2.7 version from https://www.dropbox.com/s/vulnv2pkbv8q92u/aima-python_python_v27_r001.zip?dl=0) or for Python 3.x get it from https://github.com/aimacode/aima-python
    DEFAULT_MISSION_TYPE = 'small'  #HINT: Choose between {small,medium,large}
    DEFAULT_MISSION_SEED_MAX = 1    #HINT: How many different instances of the given mission (i.e. maze layout)
    DEFAULT_REPEATS      = 1        #HINT: How many repetitions of the same maze layout
    DEFAULT_PORT         = 0
    DEFAULT_SAVE_PATH    = './results/'

    #-- Import required modules --#
    import sys
    import time
    import random
    from random import shuffle
    import argparse
    import pickle
    import datetime
    from copy import deepcopy
    import hashlib
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    import networkx as nx
    from matplotlib import lines

    #-- Define the commandline arguments required to run the agents from command line --#
    parser = argparse.ArgumentParser()
    parser.add_argument("-a" , "--agentname"        , type=str, help="path for the malmo pyhton examples"   , default=DEFAULT_AGENT_NAME)
    parser.add_argument("-t" , "--missiontype"      , type=str, help="mission type (small,medium,large)"    , default=DEFAULT_MISSION_TYPE)
    parser.add_argument("-s" , "--missionseedmax"   , type=int, help="maximum mission seed value (integer)"           , default=DEFAULT_MISSION_SEED_MAX)
    parser.add_argument("-n" , "--nrepeats"         , type=int, help="repeat of a specific agent (if stochastic behavior)"  , default=DEFAULT_REPEATS)
    parser.add_argument("-g" , "--studentguid"      , type=str, help="student guid"                         , default=DEFAULT_STUDENT_GUID)
    parser.add_argument("-p" , "--malmopath"        , type=str, help="path for the malmo pyhton examples"   , default=DEFAULT_MALMO_PATH)
    parser.add_argument("-x" , "--malmoport"        , type=int, help="special port for the Minecraft client", default=DEFAULT_PORT)
    parser.add_argument("-o" , "--aimapath"         , type=str, help="path for the aima toolbox (optional)"   , default=DEFAULT_AIMA_PATH)
    parser.add_argument("-r" , "--resultpath"       , type=str, help="the path where the results are saved" , default=DEFAULT_SAVE_PATH)
    args = parser.parse_args()
    print args

    #-- Display infor about the system --#
    print("Working dir: "+os.getcwd())
    print("Python version: "+sys.version)
    print("malmopath: "+args.malmopath)
    print("JAVA_HOME:' "+os.environ["JAVA_HOME"]+"'")
    print("MALMO_XSD_PATH: '"+os.environ["MALMO_XSD_PATH"]+"'")

    #-- Add the Malmo path  --#
    print('Add Malmo Python API/lib to the Python environment ['+args.malmopath+'Python_Examples'+']')
    sys.path.append(args.malmopath+'Python_Examples/')

    #-- Import the Malmo Python wrapper/module --#
    print('Import the Malmo module...')
    import MalmoPython

    #-- OPTIONAL: Import the AIMA tools (for representing the state-space)--#
    print('Add AIMA lib to the Python environment ['+args.aimapath+']')
    sys.path.append(args.aimapath+'/')
    from search import *

    #-- Create the command line string for convenience --#
    cmd = 'python myagents.py -a ' + args.agentname + ' -s ' + str(args.missionseedmax) + ' -n ' + str(args.nrepeats) + ' -t ' + args.missiontype + ' -g ' + args.studentguid + ' -p ' + args.malmopath + ' -x ' + str(args.malmoport)
    print(cmd)

    #-- Run the agent a number of times (it only makes a difference if you agent has some random elemnt to it, initalizaiton, behavior, etc.) --#
    #-- HINT: It is quite important that you understand the need for the loops  --#
    #-- HINT: Depending on how you implement your realistic agent in terms of restarts and repeats, you may want to change the way the loops operate --#

    print('Instantiate an agent interface/api to Malmo')
    agent_host = MalmoPython.AgentHost()

    #-- Itereate a few different layout of the same mission stype --#
    for i_training_seed in range(0,args.missionseedmax):

        #-- Observe the full state space a prior i (only allowed for the simple agent!) ? --#
        if args.agentname.lower()=='simple':
            print('Get state-space representation using a AgentHelper...[note in v0.30 there is now an faster way of getting the state-space ]')
            helper_solution_report = SolutionReport()
            helper_agent = AgentHelper(agent_host,args.malmoport,args.missiontype,i_training_seed, helper_solution_report, None)
            helper_agent.run_agent()
        else:
            helper_agent = None

        #-- Repeat the same instance (size and seed) multiple times --#
        for i_rep in range(0,args.nrepeats):
            print('Setup the performance log...')
            solution_report = SolutionReport()
            solution_report.setStudentGuid(args.studentguid)

            print('Get an instance of the specific ' + args.agentname + ' agent with the agent_host and load the ' + args.missiontype + ' mission with seed ' + str(i_training_seed))
            agent_name = 'Agent' + args.agentname
            state_space = None;
            if not helper_agent==None:
                state_space = deepcopy(helper_agent.state_space)

            agent_to_be_evaluated = eval(agent_name+'(agent_host,args.malmoport,args.missiontype,i_training_seed,solution_report,state_space)')

            print('Run the agent, time it and log the performance...')
            solution_report.start() # start the timer (may be overwritten in the agent to provide a fair comparison)
            agent_to_be_evaluated.run_agent()
            solution_report.stop() # stop the timer

            print("\n---------------------------------------------")
            print("| Solution Report Summary: ")
            print("|\tCumulative reward = " + str(solution_report.reward_cumulative))
            print("|\tDuration (wallclock) = " + str((solution_report.end_datetime_wallclock-solution_report.start_datetime_wallclock).total_seconds()))
            print("|\tNumber of reported actions = " + str(solution_report.action_count))
            print("|\tFinal goal reached = " + str(solution_report.is_goal))
            print("|\tTimeout = " + str(solution_report.is_timeout))
            print("---------------------------------------------\n")

            print('Save the solution report to a specific file for later analysis and reporting...')
            fn_result = args.resultpath + 'solution_' + args.studentguid + '_' + agent_name + '_' +args.missiontype + '_' + str(i_training_seed) + '_' + str(i_rep)
            foutput = open(fn_result+'.pkl', 'wb')
            pickle.dump(agent_to_be_evaluated.solution_report,foutput) # Save the solution information in a specific file, HiNT:  It can be loaded with pickle.load(output) with read permissions to the file
            foutput.close()

            # You can reload the results for this instance using...
            #finput = open(fn_result+'.pkl', 'rb')
            #res =  pickle.load(finput)
            #finput.close()

            print('Sleep a sec to make sure the client is ready for next mission/agent variation...')
            time.sleep(1)
            print("------------------------------------------------------------------------------\n")

    print("Done")
    print(agent_host)
