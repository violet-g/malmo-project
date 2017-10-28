import datetime

class SolutionReport(object):
    # Please do not modify this custom class !
    def __init__(self):
        """ Constructor for the solution info
        class which is used to store a summary of
        the mission and agent performance """

        self.start_datetime_wallclock = None;
        self.end_datetime_wallclock = None;
        self.action_count = 0;
        self.reward_cumulative = 0;
        self.mission_xml = None;
        self.mission_type = None;
        self.mission_seed = None;
        self.student_guid = None;
        self.mission_xml_as_expected = None;
        self.is_goal = None;
        self.is_timeout = None;

    def start(self):
        self.start_datetime_wallclock = datetime.datetime.now()

    def stop(self):
        self.checkGoal()
        self.end_datetime_wallclock = datetime.datetime.now()

    def setMissionXML(self, mission_xml):
        self.mission_xml = mission_xml

    def setMissionType(self, mission_type):
        self.mission_type = mission_type

    def setMissionSeed(self, mission_seed):
        self.mission_seed = mission_seed

    def setStudentGuid(self, student_guid):
        self.student_guid = student_guid

    def addAction(self):
        self.action_count += 1

    def addReward(self, reward, datetime_):
        self.reward_cumulative += reward

    def checkMissionXML(self):
          """ This function is part of the final check"""
          #-- It is not included for simplifity --#
          self.mission_xml_as_expected = 'Unknown'

    def checkGoal(self):
          """ This function checks if the goal has been reached based on the expected reward structure (will not work if you change the mission xml!)"""
          #-- It is not included for simplifity --#
          if self.reward_cumulative!=None:
            x = round((abs(self.reward_cumulative)-abs(round(self.reward_cumulative)))*100);
            rem_goal = x%7
            rem_timeout = x%20
            if rem_goal==0 and x!=0:
                self.is_goal = True
            else:
                self.is_goal = False

            if rem_timeout==0 and x!=0:
                self.is_timeout = True
            else:
                self.is_timeout = False
