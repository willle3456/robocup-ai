from path_planner import PathPlanner, Point

class PRM(PathPlanner):
    def __init__(self, num_samples=200, num_neigh=5):
        super(PRM, self).__init__()
        self.num_samples = num_samples
        self.num_neigh = num_neigh

    def run(self):
        self.graph.clear()
        self.sample_space()
        self.build_road_map()
        while not self.found_path():
            self.sample_space()
            self.build_road_map()
        self.found_goal = True
        self.goal_node = self.goal
        #return path
        #get path in the viz for now

    def sample_space(self):
        i = 0
        while i < self.num_samples:
            q = self.get_random_pt()
            while self.point_collision(q):
                q = self.get_random_pt()
            self.graph.add_node(q)
            i += 1

    def build_road_map(self):
        for node in list(self.graph.nodes()):
            neighbors = self.get_nearest_neighbors(node)
            for n in neighbors:
                if (not self.graph.has_edge(node, n)) and (not self.line_collision(node, n)):
                        self.graph.add_edge(node, n, drawn='False')

    def found_path(self):
        
        path_to_start = False
        path_to_goal = False
        
        # connect the goal and start to the found path
        s_nn = self.get_nearest_neighbors(self.start)
        g_nn = self.get_nearest_neighbors(self.goal)
        
        if not self.graph.has_node(self.start):
            for n in s_nn:
                if not self.line_collision(n, self.start):
                    self.graph.add_edge(n, self.start, drawn='False')
                    path_to_start = True
                    break
        else:
            path_to_start = True

        if not self.graph.has_node(self.goal):
            for n in g_nn:
                if not self.line_collision(n, self.goal):
                    self.graph.add_edge(n, self.goal, drawn='False')
                    path_to_goal = True
                    break
        else:
            path_to_goal = True
        
        return path_to_start and path_to_goal

    def get_nearest_neighbors(self, q):
        nn = {Point(0, 0) : self.max_dist for i in range(self.num_neigh)}
        for node in list(self.graph.nodes()):
            for key, value in nn.iteritems():
                tmp_dist = q.dist(node)
                if (value > tmp_dist) and q != node:
                    nn[node] = tmp_dist
                    del nn[key]
                    break
        nn_sort = sorted(nn, key=nn.get)
        return nn.keys()
