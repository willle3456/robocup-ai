from path_planner import PathPlanner, Point
from inspect import currentframe, getframeinfo
import numpy as np

def print_all(f_name, l_no, msg):
    print '{0} :{1}, line {2}'.format(msg, f_name, l_no)

class RRT(PathPlanner):
    def __init__(self, step_size=50, bot_size=150, num_iter=25):
        super(RRT, self).__init__()
        self.step_size = step_size
        self.bot_size = bot_size
        self.num_iter = num_iter

    def run(self):
        self.graph.clear()
        self.graph.add_node(self.start)
        self.found_goal = False
        #self.graph.goal_node = None
        while not self.found_goal:
            for i in range(self.num_iter):
                q_rand = self.get_random_pt()
                frameinfo = getframeinfo(currentframe())
                q_new = self.extend(q_rand)
                if q_new is not None:
                    if q_new.dist(self.goal) < (3 * self.bot_size):
                        self.found_goal = True
                        self.goal_node = q_new
                        break
    
    def extend(self, q_rand):
        q_near = self.get_nearest(q_rand)
        q_new_vector = q_rand.convert_to_array() - q_near.convert_to_array()
        q_new = self.step_size * (q_new_vector / np.linalg.norm(q_new_vector))
        q_new += q_near.convert_to_array()
    
        q_new_pt = Point(q_new[0], q_new[1])
        if self.point_collision(q_new_pt):
            return None
        self.graph.add_node(q_new_pt)
        self.graph.add_edge(q_near, q_new_pt)
        return q_new_pt

    def get_nearest(self, q_rand):
        closest = self.graph.nodes().items()[0][0]
        closest_dist = self.max_dist         
        for node in list(self.graph.nodes()):
            tmp_dist = q_rand.dist(node)
            if tmp_dist < closest_dist:
                closest = node
                closest_dist = tmp_dist
        return closest
