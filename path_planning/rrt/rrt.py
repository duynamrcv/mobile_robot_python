import math
import random

import numpy as np
import matplotlib.pyplot as plt

show_animation = True

class RRT:
    ''' Class for RRT planning '''
    class Node:
        def __init__(self, x, y):
            self.x = x; self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
    
    def __init__(self, start, goal, obs_list, rand_area, expand_dis=1.0,
                path_resolution=0.5, goal_sample_rate=20, max_iter=500):
        '''
        start       Start Position [x,y]
        goal        Goal Position [x,y]
        obs_list    Obstacle Positions [[x,y,size],...]
        rand_area   Random Sampling Area [min,max]
        '''
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obs_list
        self.node_list = []

    def planning(self, animation=True):
        ''' RRT planning '''
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rand_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rand_node, self.expand_dis)
            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rand_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)
            
            if animation and i % 5:
                self.draw_graph(rand_node)
        return None

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rand = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:
            rand = self.Node(self.end.x, self.end.y)
        return rand
    
    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        dis, theta = self.calc_distance_and_angle(new_node, to_node)
        
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > dis:
            extend_length = dis

        n_expand = math.floor(extend_length/self.path_resolution)

        for i in range(n_expand):
            new_node.x += self.path_resolution*math.cos(theta)
            new_node.y += self.path_resolution*math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        dis, theta = self.calc_distance_and_angle(new_node, to_node)
        if dis <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y
        new_node.parent = from_node
        return new_node
    
    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)
    
    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def draw_graph(self, rand=None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                    lambda event: [exit(0) if event.key == 'escape' else None])
        if rand is not None:
            plt.plot(rand.x, rand.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def get_nearest_node_index(node_list, rand_node):
        dlist = [(node.x - rand_node.x)**2 + (node.y - rand_node.y)**2 for node in node_list]
        min_ind = dlist.index(min(dlist))
        return min_ind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dis = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return dis, theta

    @staticmethod
    def check_collision(node, obs_list):
        if node is None:
            return False

        for (ox, oy, size) in obs_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx**2 + dy**2 for (dx, dy) in zip(dx_list, dy_list)]
            
            if min(d_list) <= size**2:
                return False    # Collision
        return True     # Safe

    @staticmethod
    def plot_circle(x, y, size, color='-b'):
        deg = list(range(0, 360, 5))
        deg.append(0)
        x1 = [x + size*math.cos(np.deg2rad(d)) for d in deg]
        y1 = [y + size*math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(x1, y1, color)

if __name__ == "__main__":
    print("Start")

    # List obstacles [x, y, radius]
    obs_list = [(5, 5, 1), (3, 6, 1), (3, 8, 1), (3, 10, 1), (7, 5, 1), (9, 5, 1), (8, 10, 1), (3.5, 5, 0.5)]

    sx = 0.0; sy = 0.0      # Start position
    gx = 6.0; gy = 10.0     # Goal position

    # Init parameters
    rrt = RRT(start=[sx,sy], goal=[gx, gy], rand_area=[-2, 15], obs_list=obs_list)
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find the path")
    else:
        print("Found the path")

        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x,y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.show()