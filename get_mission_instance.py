import random

def GetMissionInstance( mission_type, mission_seed, agent_type):
    """ Creates a specific instance of a given mission type """

    #Size of the problem
    msize = {
        'small': 10,
        'medium': 20,
        'large': 40,
    }

    # Timelimit
    mtimeout = {
        'small':   60000,
        'medium': 240000,
        'large':  960000,
        'helper': 200000,
    }

    # Number of intermediate rewards
    nir = {
        'small': 3,
        'medium': 9,
        'large': 27,
    }

    mission_type_tmp = mission_type
    if agent_type.lower()=='helper':
        mission_type_tmp = agent_type.lower()

    #-- Define various parameters used in the generation of the mission --#
    #-- HINT: It is crucial that you understand the details of the mission, this will require some knowledge of uncertainty/probability and random variables --#
    random.seed(mission_seed)
    reward_goal = abs(round(random.gauss(1000, 400)))+0.0700
    reward_waypoint = round(abs(random.gauss(3, 15)))
    reward_timeout = -round(abs(random.gauss(1000, 400)))-0.2000
    reward_sendcommand = round(-random.randrange(2,10))

    n_intermediate_rewards = random.randrange(1,5) * nir.get(mission_type, 10) # How many intermediate rewards...?

    xml_str = '''<?xml version="1.0" encoding="UTF-8" ?>
        <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
          <About>
            <Summary>Mission for assessed exercise 2016/2017 University of Glasgow</Summary>
          </About>
          <ServerSection>
            <ServerInitialConditions>
              <Time>
                <StartTime>6000</StartTime>
                <AllowPassageOfTime>false</AllowPassageOfTime>
              </Time>
              <Weather>clear</Weather>
              <AllowSpawning>false</AllowSpawning>
            </ServerInitialConditions>
            <ServerHandlers>
              <FlatWorldGenerator generatorString="3;7,220*1,5*3,2;3;,biome_1" />
              <MazeDecorator>
                <Seed>'''+str(mission_seed)+'''</Seed>
                <SizeAndPosition length="'''+str(msize.get(mission_type,100))+'''" width="'''+str(msize.get(mission_type,100))+'''" yOrigin="215" zOrigin="0" xOrigin="0" height="180"/>
                <GapProbability variance="0.3">0.2</GapProbability>
                <MaterialSeed>1</MaterialSeed>
                <AllowDiagonalMovement>false</AllowDiagonalMovement>
                <StartBlock fixedToEdge="true" type="emerald_block" height="1"/>
                <EndBlock fixedToEdge="false" type="redstone_block" height="12"/>
                <PathBlock type="glowstone" colour="WHITE ORANGE MAGENTA LIGHT_BLUE YELLOW LIME PINK GRAY SILVER CYAN PURPLE BLUE BROWN GREEN RED BLACK" height="1"/>
                <FloorBlock type="air"/>
                <SubgoalBlock type="glowstone"/>
                <GapBlock type="stained_hardened_clay" colour="WHITE ORANGE MAGENTA LIGHT_BLUE YELLOW LIME PINK GRAY SILVER CYAN PURPLE BLUE BROWN GREEN RED BLACK" height="3"/>
                <Waypoints quantity="'''+str(n_intermediate_rewards)+'''">
                  <WaypointItem type="diamond_block"/>
                </Waypoints>
              </MazeDecorator>
              <ServerQuitFromTimeUp timeLimitMs="'''+str(mtimeout.get(mission_type_tmp,0))+'''" description="out_of_time"/>
              <ServerQuitWhenAnyAgentFinishes />
            </ServerHandlers>
          </ServerSection>
          <AgentSection>
            <Name>My Agent</Name>
            <AgentStart>
              <Placement x="0" y="216" z="90"/> <!-- will be overwritten by MazeDecorator -->
            </AgentStart>
            <AgentHandlers>
              <VideoProducer want_depth="false">
              <Width>1280</Width>
              <Height>960</Height>
              </VideoProducer>
              <ObservationFromFullStats/>
              <ObservationFromRecentCommands/>
              <ObservationFromFullInventory/>
              <RewardForCollectingItem>
                <Item reward="'''+str(reward_waypoint)+'''" type="diamond_block"/>
              </RewardForCollectingItem>
              <RewardForSendingCommand reward="'''+str(reward_sendcommand)+'''"/>
              <RewardForMissionEnd rewardForDeath="-1000000">
                <Reward description="found_goal" reward="'''+str(reward_goal)+'''" />
                <Reward description="out_of_time" reward="'''+str(reward_timeout)+'''" />
              </RewardForMissionEnd>
              <AgentQuitFromTouchingBlockType>
                <Block type="redstone_block" description="found_goal" />
              </AgentQuitFromTouchingBlockType>
            </AgentHandlers>
          </AgentSection>
        </Mission>'''
    return xml_str, msize.get(mission_type,100),reward_goal,reward_waypoint,n_intermediate_rewards,reward_timeout,reward_sendcommand,mtimeout.get(mission_type_tmp,0)
