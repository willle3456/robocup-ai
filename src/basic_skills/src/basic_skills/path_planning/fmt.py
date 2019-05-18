from prm import PRM

class FMT(PRM):
    def __init__(self, num_samples=200, radius=150):
        super(FMT, self).__init__()
        self.num_samples = num_samples
        self.radius = radius

    def run(self):
        self.graph.clear()
        self.sample_space()
        self.graph.add_node(self.start)
        self.graph.add_node(self.goal)
        v_unvisited = [v for v in list(self.graph.nodes()) if v != self.start]
        v_open = [self.start]
        v_closed = []
        z = self.start
        #nn_z.append(z)
        # the dict helps neighbor finding
        #nn_z_dict = {z: nn_z}
        cost = {v: 0 for v in list(self.graph.nodes())}
        while z != self.goal:
            v_open_new = []
            nn_z = self.get_neighbors_in_range(z)
            x_near = [v for v in v_unvisited if v in nn_z]
            for x in x_near:
                nn_x = self.get_neighbors_in_range(x)
                #nn_x.append(x)
                #nn_x_dict = {x: nn_x}
                y_near = [v for v in v_open if v in nn_x]
                y_min = self.lowest_cost(cost, y_near, x)
                if not self.line_collision(y_min, x):
                    self.graph.add_edge(y_min, x)
                    v_open_new.append(x)
                    v_unvisited.remove(x)
                    cost[x] = cost[y_min] + self.get_cost(y_min, x)
            v_open.extend(v_open_new)
            v_open.remove(z)
            v_closed.append(z)
            if not v_open:
                'print no path found'
                return
            z = self.lowest_cost(cost, v_open)
        self.found_goal = True
        self.goal_node = self.goal
    
    def get_neighbors_in_range(self, pt):
        neighbors = []
        for node in list(self.graph.nodes()):
            if pt.dist(node) <= self.radius:
                neighbors.append(node)
        return neighbors

    def get_cost(self, pt_1, pt_2):
        return pt_1.dist(pt_2)

    def lowest_cost(self, cost, pt_list, pt=None):
        node = pt_list[0]
        min_cost = cost[node]
        if pt:
            min_cost += pt.dist(pt_list[0])
        for n in pt_list:
            tmp_cost = cost[n]
            if pt:
                tmp_cost += pt.dist(n)
            if tmp_cost < min_cost:
                node = n
        return node
        

