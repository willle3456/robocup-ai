from action import Action, CommandStatus
from player_data import PlayerData, Pose2D

class Robot(object):

    """
    Decodes and renders robot packets from grSim, and manages actions to send back to grSim for a specific robot
    """
    def __init__(self, id_num, num_robots=6, is_blue=True):
        """Initializes a robot object is_blue - boolean idNum - int game - grsim"""
        self.is_blue = is_blue
        self.num_robots = num_robots
        self.id = id_num
        self.ego_data = PlayerData(id_num)
        self.friends = {i: PlayerData(i) for i in range(self.num_robots) if i != self.id}
        self.enemies = {i: PlayerData(i) for i in range(self.num_robots)}
    
        self.ball = PlayerData()
        self._default_action = None
        self._waiting_action = None
        self._action = None

    def __str__(self):
        return 'Robot Data:\nTeam Color: {0}\nPlayer Data: {1}\nTeam Data: {2}\nOpp Data: {3}\nDefault Action: {4}\nNext Action: {5}\nCurrent Action: {6}\n'.format(self.is_blue, self.ego_data, self.friends, self.enemies, self._default_action, self._waiting_action, self._action)

    def get_cart_location(self):
        return self.ego_data.location.convert_to_array()

    def get_orientation(self):
        return self.ego_data.location.theta

    def get_cart_velocity(self):
        return self.ego_data.velocities.convert_to_array()

    def get_angular_velocity(self):
        return self.ego_data.velocities.theta 

    def get_ball_location(self):
        return self.ball.location.convert_to_array()

    def get_ball_velocity(self):
        return self.ball.velocities.convert_to_array()

    def get_action (self):
        return self._action
    
    def run_action(self, delta_time):
        """
        Checks to see if there are any state changes to the active action, then updates the current action
        :param delta_time: The time passed since the last update
        """
       # print(88, self._waiting_action, self._action)
        action_result = [0]*5
        
        if self._waiting_action is not None:
        # If a new action is waiting to be executed, interrupt any current actions and start execution of
        # the new one
            if self._action is not None:
                self._action.end(CommandStatus.INTERRUPTED)

            self._action = self._waiting_action
            self._waiting_action = None
            self._action.start()

        if self._action is not None:
        # Update the current action if valid
            status = self._action.get_status()

            if status == CommandStatus.RUNNING:
                action_result = self._action.run(delta_time)
            else:
                self._action.end(status)
                self._action = None

        if self._action is None and self._waiting_action is None and self._default_action is not None:
        # If there is not action running and no actions are waiting, run the default action
            self._action = self._default_action
            self._action.start()
      
        return action_result
    
    def set_default_action(self, action):
        """
        Sets the robot's default action - the action will be executed if all other actions have terminated
        :param action: The action to be defined as the default
        """
        if action.get_robot() == self or action.set_robot(self):
            self._default_action = action
  
    def add_action(self, action):
        """
        Runs the given action on the robot
        :param action: The action to run on the robot
        """
        if action.get_robot() == self or action.set_robot(self) or self.is_virtual_robot:
            if self._action is not None:
                self._waiting_action = action
            else:
                self._action = action
                action.add(self)
