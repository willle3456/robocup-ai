from path_planner import PathPlanner
from basic_skills.player_data import Pose2D
from inspect import currentframe, getframeinfo
import numpy as np
import collision_checker

def print_all(f_name, l_no, msg):
    print '{0} :{1}, line {2}'.format(msg, f_name, l_no)

class RRT(PathPlanner):
    def __init__(self, step_size=50, bot_size=150, num_iter=25):
        super(RRT, self).__init__()
        self.step_size = step_size
        self.bot_size = bot_size
        self.num_iter = num_iter

    def plan(self):
        #self.graph.clear()
        self.graph_data.new_nodes = []
        self.graph_data.removed_nodes = []
        self.graph_data.new_edges = []
        self.graph_data.removed_edges = []
        self.graph.add_node(self.start)
        self.found_goal = False
        #self.graph.goal_node = None

        while not self.found_goal:
            for i in range(self.num_iter):
                q_rand = self.sample()
                frameinfo = getframeinfo(currentframe())
                q_new = self.extend(q_rand)

                if q_new is not None:
                    if q_new.get_distance(self.goal) < (3 * self.bot_size):
                        self.found_goal = True
                        self.goal_node = q_new
                        break
    
    def extend(self, q_rand):
        q_near = self.get_nearest(q_rand)
        q_new_vector = q_rand.convert_to_array() - q_near.convert_to_array()
        q_new = self.step_size * (q_new_vector / np.linalg.norm(q_new_vector))
        q_new += q_near.convert_to_array()
    
        q_new_pt = Pose2D(q_new[0], q_new[1], 0)

        if collision_checker.is_collision(q_new_pt, None, self.obstacles):
            return None

        self.graph.add_node(q_new_pt)
        self.graph.add_edge(q_near, q_new_pt)
        
        self.graph_data.new_nodes.append(q_new_pt.convert_to_pose())
        self.graph_data.new_edges.append(q_near.convert_to_pose())
        self.graph_data.new_edges.append(q_new_pt.convert_to_pose())

        return q_new_pt

    def get_nearest(self, q_rand):
        closest = self.graph.nodes().items()[0][0]
        closest_dist = self.max_dist         
        
        for node in list(self.graph.nodes()):
            tmp_dist = q_rand.get_distance(node)
            
            if tmp_dist < closest_dist:
                closest = node
                closest_dist = tmp_dist

        return closest
