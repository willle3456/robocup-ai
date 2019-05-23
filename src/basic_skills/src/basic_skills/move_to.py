#!/usr/env/bin  python
from action import Action
from path_planning.rrt import RRT
from path_planning.planner_plotter import PlannerPlotter
from path_planning.path_planner import Point

class MoveTo(Action):
    def __init__(self):
        super(MoveTo, self).__init__()
        self.planner = RRT(step_size=200, num_iter=600)
        self.plotter = PlannerPlotter(self.planner)

    def set_goal(self, goal):
        self.planner.goal = goal
    
    def get_players_as_obs(self):
        obs = []
        for k,f in self._robot.friends.iteritems():
            o = Point(f.location.x, f.location.y)
            obs.append(o)
            
        for k,e in self._robot.enemies.iteritems():
            o = Point(e.location.x, e.location.y)
            obs.append(o)

        return obs
    
    def run_loop(self):
        obs = self.get_players_as_obs()
        self.planner.update_obstacles(obs)
        self.planner.run()
        self.plotter.draw()

    def start(self):
        # need to get graph here
        self.planner.start = self._robot.ego_data.location.convert_to_point()
        self.run_loop()

    def run(self, delta_time):
        self.planner.start = self._robot.ego_data.location.convert_to_point()
        self.run_loop()

    def end(self, status):
        self.planner.start = None
        self.planner.goal = None
        self.plotter.close()
