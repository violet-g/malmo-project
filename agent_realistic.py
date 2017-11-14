import time
import random
import json

from agent_helper import AgentHelper
from init_mission import init_mission

import MalmoPython

# This class implements the Realistic Agent --#
class AgentRealistic:

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the realistic agent """
        self.AGENT_MOVEMENT_TYPE = 'Absolute' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Realistic'
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = None; # NOTE: The Realistic can not know anything about the state_space a priori !
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)
        self.actions = []
		self.q_table = {}

    #----------------------------------------------------------------------------------------------------------------#
    def __ExecuteActionForRealisticAgentWithNoisyTransitionModel__(idx_requested_action, noise_level):
        """ Creates a well-defined transition model with a certain noise level """
        n = len(self.AGENT_ALLOWED_ACTIONS)
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_request_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distribution of actions
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]
        self.agent_host.sendCommand(actual_action)
        return actual_action

    def act_agent(self, world_state, agent_host, reward_cumulative):
        """ Take an action in response to the current world state"""
        
        obs_text = world_state.observations[-1].text
        obs = json.loads(obs_text) # most recent observation
        self.logger.debug(obs)
        if not u'XPos' in obs or not u'ZPos' in obs:
            self.logger.error("Incomplete observation received: %s" % obs_text)
            return 0
        current_s = "%d:%d" % (int(obs[u'XPos']), int(obs[u'ZPos']))
        self.logger.debug("State: %s (x = %.2f, z = %.2f)" % (current_s, float(obs[u'XPos']), float(obs[u'ZPos'])))
        if not self.q_table.has_key(current_s):
            self.q_table[current_s] = ([0] * len(self.actions))

        # update Q values
        if self.training and self.prev_s is not None and self.prev_a is not None:
            old_q = self.q_table[self.prev_s][self.prev_a]
            self.q_table[self.prev_s][self.prev_a] = old_q + self.alpha * (reward_cumulative
                + self.gamma * max(self.q_table[current_s]) - old_q)

        # select the next action
        m = max(self.q_table[current_s])
        self.logger.debug("Current values: %s" % ",".join(str(x) for x in self.q_table[current_s]))
        l = list()
        for x in range(0, len(self.actions)):
            if self.q_table[current_s][x] == m:
                l.append(x)
        y = random.randint(0, len(l)-1)
        a = l[y]
        self.logger.info("Taking q action: %s" % self.actions[a])

        # send the selected action
        agent_host.ExecuteActionForRealisticAgentWithNoisyTransitionModel(self.actions[a], 0.0) # TODO add noise for submission
        self.prev_s = current_s
        self.prev_a = a

        return reward_cumulative

    #----------------------------------------------------------------------------------------------------------------#
    def run_agent(self):
        """ Run the Realistic agent and log the performance and resource use """

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
		

		### INPUTS
        # Wait for a valid observation
        state_space_locations = self.state_space.state_locations
        print(state_space_locations)

        continuousMovement = False
		
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
		#self.agent_host.setRewardsPolicy()
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)

        # Goal:
        # goal_t: The goal is obtained when the cumulative reward reaches 1000 (checked internally in the mission definition)
        # Let's predefine the cumulative reward - note the goal test is (effectively) checked against this value
        reward_cumulative = 0.0
        total_reward = 0
		tol = 0

        self.prev_s = None
        self.prev_a = None

        # wait for a valid observation
        world_state = agent_host.peekWorldState()
        while world_state.is_mission_running and all(e.text=='{}' for e in world_state.observations):
            world_state = agent_host.peekWorldState()
        # wait for a frame to arrive after that
        num_frames_seen = world_state.number_of_video_frames_since_last_state
        while world_state.is_mission_running and world_state.number_of_video_frames_since_last_state == num_frames_seen:
            world_state = agent_host.peekWorldState()
        world_state = agent_host.getWorldState()
        for err in world_state.errors:
            print err

        assert len(world_state.video_frames) > 0, 'No video frames!?'
        
        obs = json.loads( world_state.observations[-1].text )
        prev_x = obs[u'XPos']
        prev_z = obs[u'ZPos']
        print 'Initial position:',prev_x,',',prev_z
            
        # take first action
        total_reward += ExecuteActionForRealisticAgentWithNoisyTransitionModel(, 0.0)
        total_reward += self.act(world_state,agent_host, reward_cumulative)
        
        require_move = True
        check_expected_position = True

		#get state_t 
        state_t = self.agent_host.getWorldState()

		#Main Loop
        while state_t.is_mission_running:

			# wait for the position to have changed and a reward received
            print 'Waiting for data...',
            while True:
                world_state = agent_host.peekWorldState()
                if not world_state.is_mission_running:
                    print 'mission ended.'
                    break
                if len(world_state.rewards) > 0 and not all(e.text=='{}' for e in world_state.observations):
                    obs = json.loads( world_state.observations[-1].text )
                    curr_x = obs[u'XPos']
                    curr_z = obs[u'ZPos']
                    if require_move:
                        if math.hypot( curr_x - prev_x, curr_z - prev_z ) > tol:
                            print 'received.'
                            break
                    else:
                        print 

			# wait for a frame to arrive after that
            num_frames_seen = world_state.number_of_video_frames_since_last_state
            while world_state.is_mission_running and world_state.number_of_video_frames_since_last_state == num_frames_seen:
                world_state = agent_host.peekWorldState()
                
            num_frames_before_get = len(world_state.video_frames)
            
            world_state = agent_host.getWorldState()
            for err in world_state.errors:
                print err
            reward_cumulative = sum(r.getValue() for r in world_state.rewards)

			if world_state.is_mission_running:
                assert len(world_state.video_frames) > 0, 'No video frames!?'
                num_frames_after_get = len(world_state.video_frames)
                assert num_frames_after_get >= num_frames_before_get, 'Fewer frames after getWorldState!?'
                frame = world_state.video_frames[-1]
                obs = json.loads( world_state.observations[-1].text )
                curr_x = obs[u'XPos']
                curr_z = obs[u'ZPos']
                print 'New position from observation:',curr_x,',',curr_z,'after action:',self.actions[self.prev_a], #NSWE
                if check_expected_position:
                    expected_x = prev_x + [0,0,-1,1][self.prev_a]
                    expected_z = prev_z + [-1,1,0,0][self.prev_a]
                    if math.hypot( curr_x - expected_x, curr_z - expected_z ) > tol:
                        print ' - ERROR DETECTED! Expected:',expected_x,',',expected_z
                        raw_input("Press Enter to continue...")
                    else:
                        print 'as expected.'
                    curr_x_from_render = frame.xPos
                    curr_z_from_render = frame.zPos
                    print 'New position from render:',curr_x_from_render,',',curr_z_from_render,'after action:',self.actions[self.prev_a], #NSWE
                    if math.hypot( curr_x_from_render - expected_x, curr_z_from_render - expected_z ) > tol:
                        print ' - ERROR DETECTED! Expected:',expected_x,',',expected_z
                        raw_input("Press Enter to continue...")
                    else:
                        print 'as expected.'
                else:
                    print
                prev_x = curr_x
                prev_z = curr_z
                # act
                total_reward += self.act(world_state, agent_host, reward_cumulative)

		#-- Print some of the state information --#
        print("Percept: video,observations,rewards received:",state_t.number_of_video_frames_since_last_state,state_t.number_of_observations_since_last_state,state_t.number_of_rewards_since_last_state)
        print("\tcoordinates (x,y,z):" + str(xpos) + " " + str(ypos) + " " + str(zpos))
		return



		# process final reward
        self.logger.debug("Final reward: %d" % reward_cumulative)
        total_reward += reward_cumulative

        # update Q values
        if self.training and self.prev_s is not None and self.prev_a is not None:
            old_q = self.q_table[self.prev_s][self.prev_a]
            self.q_table[self.prev_s][self.prev_a] = old_q + self.alpha * ( reward_cumulative - old_q )
            


			#Set the world state
            state_t = self.agent_host.getWorldState()

			# Stop movement
            """if state_t.is_mission_running:
                # Enforce a simple discrete behavior by stopping any continuous movement in progress

                if continuousMovement:
                    self.agent_host.sendCommand("move "  + str(0))
                    self.agent_host.sendCommand("pitch " + str(0))
                    self.agent_host.sendCommand("turn "  + str(0))
                else:
                    actionIdx = random.randint(0, 2)
                    self.agent_host.sendCommand(discreteAction[actionIdx])

			#"""
        return
