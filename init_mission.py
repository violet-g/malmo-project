from get_mission_instance import GetMissionInstance
import MalmoPython
import sys
import time

# This function initialized the mission based on the input arguments
def init_mission(agent_host, port=0, agent_type='Unknown',mission_type='Unknown', mission_seed=0, movement_type='Continuous'):
    """ Generate, and load the mission and return the agent host """

    #-- Set up the mission via XML definition --#
    mission_xml, msize, reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = GetMissionInstance(mission_type,mission_seed,agent_type)
    my_mission = MalmoPython.MissionSpec(mission_xml, True)
    my_mission.forceWorldReset()

    #-- Enforce the specific restriction for the assessed exercise --#
    #-- If you want a super agent, define one for you self  --#
    my_mission.setModeToCreative()
    if agent_type.lower()=='random':
        n = msize
        my_mission.observeGrid(-n, -1, -n, n, -1, n, 'grid')
        my_mission.requestVideoWithDepth(640,480)
    elif agent_type.lower()=='simple':
        n = msize
        my_mission.observeGrid(-n, -1, -n, n, -1, n, 'grid');
        my_mission.requestVideo(640,480)
    elif agent_type.lower()=='realistic':
        n = 1 # n=1 means local info only !
        my_mission.observeGrid(-n,-1,-n, n, -1, n, 'grid');
        my_mission.requestVideoWithDepth(640,480)
    elif agent_type.lower()=='helper':
        n = 100
        my_mission.observeGrid(-n,-1,-n, n, -1, n, 'grid');
        my_mission.requestVideoWithDepth(640,480)
    else:
        #-- Define a custom agent and add the sensors you need --#
        n = 100
        my_mission.observeGrid(-n, -1, -n, n, 1, n, 'grid');
        my_mission.requestVideoWithDepth(640,480)

    #-- Add support for the specific movement type requested (and given the constraints of the assignment) --#
    #-- See e.g. http://microsoft.github.io/malmo/0.17.0/Schemas/MissionHandlers.html   --#
    if movement_type.lower()=='absolute':
        my_mission.allowAllAbsoluteMovementCommands()
    elif movement_type.lower()=='continuous':
        my_mission.allowContinuousMovementCommand('move')
        my_mission.allowContinuousMovementCommand('strafe')
        my_mission.allowContinuousMovementCommand('pitch')
        my_mission.allowContinuousMovementCommand('turn')
        my_mission.allowContinuousMovementCommand('crouch')
    elif movement_type.lower()=='discrete':
        my_mission.allowDiscreteMovementCommand('turn')
        my_mission.allowDiscreteMovementCommand('move')
        my_mission.allowDiscreteMovementCommand('movenorth')
        my_mission.allowDiscreteMovementCommand('moveeast')
        my_mission.allowDiscreteMovementCommand('movesouth')
        my_mission.allowDiscreteMovementCommand('movewest')
        my_mission.allowDiscreteMovementCommand('look')

    #-- Get the resulting xml (and return in order to check that conditions match the report) --#
    final_xml = my_mission.getAsXML(True)

    # Set up a recording for later inspection
    my_mission_record = MalmoPython.MissionRecordSpec('tmp' + ".tgz")
    my_mission_record.recordRewards()
    my_mission_record.recordMP4(24,400000)

    #-- Attempt to start a mission --#
    max_retries = 5
    for retry in range(max_retries):
        try:
            agent_host.startMission(my_mission, my_mission_record )
            break
        except RuntimeError as e:
            if retry == max_retries - 1:
                print("Error starting mission:",e)
                exit(1)
            else:
                time.sleep(2)

    #-- Loop until mission starts: --#
    print("Waiting for the mission to start ")
    state_t = agent_host.getWorldState()
    while not state_t.has_mission_begun:
        sys.stdout.write(".")
        time.sleep(0.1)
        state_t = agent_host.getWorldState()
        for error in state_t.errors:
            print("Error:",error.text)

    print
    print( "Mission started (xml returned)... ")
    return final_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand,timeout
