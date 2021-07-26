import matplotlib.pyplot as plt
import math

show_animation = True

class Dijkstra:
    def __init__(self, ox, oy, resolution, robot_radius):
        '''
        Initialize map for Dijkstra planning
        ox - x position list of Obstacles
        oy - y position list of Obstacles
        resolution - grid resolution
        robot_radius - robot radius
        '''

        self.min_x = None; self.min_y = None
        self.max_x = None; self.max_y = None
        self.x_width = None; self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x; self.y = y
            self.cost = cost
            self.parent_index = parent_index    # index of previous node
        
        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        '''
        Dijkstra path planning
        Input:  sx, sy - start position (x, y)
                gx, gy - goal position (x, y)
        Output: rx, ry - list of final path (x, y)
        '''
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                                self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                                self.calc_xy_index(gy, self.min_y), 0.0, -1)
        
        open_set, close_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while 1:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # Show graph
            if show_animation:
                plt.plot(self.calc_position(current.x, self.min_x),
                        self.calc_position(current.y, self.min_y), "xc")

                # ESC for stop simulation
                plt.gcf().canvas.mpl_connect('key_release_event',
                                            lambda event:[exit(0) if event.key == "escape" else None])
                
                if len(close_set.keys()) % 10 == 0:
                    plt.pause(0.001)
            
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Found goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
        
            # Remove the item from open set
            del open_set[c_id]

            # Add to close set
            close_set[c_id] = current

            # Expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                current.y + move_y,
                                current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in close_set:
                    continue
                if not self.verify_node(node):
                    continue
                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost >= node.cost:
                        open_set[n_id] = node
        
        rx, ry = self.calc_final_path(goal_node, close_set)
        return rx, ry

    def calc_final_path(self, goal_node, close_set):
        rx = [self.calc_position(goal_node.x, self.min_x)]
        ry = [self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index

        while parent_index != -1:
            n = close_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    def calc_position(self, index, minp):
        pos = index*self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp)/self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y)*self.x_width + (node.x-self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y:
            return False
        if px >= self.max_x or py >= self.max_y:
            return False
        
        if self.obstacle_map[node.x][node.y]:
            return False
        
        return True
    
    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        print("min_x: ", self.min_x)
        print("min_y: ", self.min_y)
        print("max_x: ", self.max_x)
        print("max_y: ", self.max_y)

        self.x_width = int(round(self.max_x - self.min_x)/self.resolution)
        self.y_width = int(round(self.max_y - self.min_y)/self.resolution)
        print("x_width: ", self.x_width)
        print("y_width: ", self.y_width)

        # Obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox-x, ioy-y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break
    
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
    
if __name__ == '__main__':
    print(__file__ + " start!!")

    # start and goal position
    sx = -5.0
    sy = -5.0
    gx = 50.0
    gy = 50.0
    grid_size = 2.0
    robot_radius = 1.0

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "or")
        plt.plot(gx, gy, "ob")
        plt.grid(True)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()
