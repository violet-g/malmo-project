#----------------------------------------------------------------------------------------------------------------#
class StateSpace(object):
    """ This is a datatype used to collect a number of important aspects of the environment
    It can be constructed online or be created offline using the Helper Agent

    You are welcome to modify or change it as you see fit

    """

    def __init__(self):
        """ Constructor for the local state-space representation derived from the Oracle"""
        self.state_locations = None
        self.state_actions = None
        self.start_id = None  # The id assigned to the start state
        self.goal_id = None   # The id assigned to the goal state
        self.start_loc = None # The real word coordinates of the start state
        self.goal_loc = None  # The real word coordinates of the goal state
        self.reward_states_n = None
        self.reward_states = None
        self.reward_sendcommand = None
        self.reward_timeout = None
        self.timeout = None
