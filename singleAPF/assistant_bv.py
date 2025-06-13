import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from matplotlib.patches import Arc, Circle, Rectangle


class MAP:

    def __init__(self, length, width, del_block):
        self.length = length
        self.width = width
        self.del_block = del_block   #The size of one grid block.
        self.no_block_x = int(length / del_block)   #Number of blocks along the x and y axes
        self.no_block_y = int(width / del_block)

    def create_map(self):
        '''Sets up an 8x8 figure using matplotlib.
        Configures labels, grid ticks, and boundaries.'''
        subplot = plt.figure(figsize=(8, 8))
        fig = subplot.add_subplot(111)
        fig.set_xlabel('X-distance (m)', size=15)
        fig.set_ylabel('Y-distance (m)', size=15)
        fig.xaxis.set_ticks_position('bottom')
        fig.yaxis.set_ticks_position('left')
        plt.tick_params(labelsize=15)
        x_major = MultipleLocator(10)
        y_major = MultipleLocator(10)
        fig.xaxis.set_major_locator(x_major)
        fig.yaxis.set_major_locator(y_major)
        plt.xlim(0, self.length)
        plt.ylim(0, self.width)

        return fig

    def draw_lines(self, fig, solid_lines, dotted_lines, transition_lines):
        for solid_line in solid_lines:
            #Drawn as solid black lines, Specifies horizontal or vertical orientation using the h or v flag.
            if solid_line[3] == 'h':
                plt.plot([solid_line[0], solid_line[1]], [solid_line[2], solid_line[2]], c='black', linewidth=2)
            else:
                plt.plot([solid_line[0], solid_line[0]], [solid_line[1], solid_line[2]], c='black', linewidth=2)

        for dotted_line in dotted_lines:
            #Drawn as dashed gray lines.
            if dotted_line[3] == 'h':
                plt.plot([dotted_line[0], dotted_line[1]], [dotted_line[2], dotted_line[2]], c='gray', linestyle='--',
                         linewidth=2)
            else:
                plt.plot([dotted_line[0], dotted_line[0]], [dotted_line[1], dotted_line[2]], c='gray', linestyle='--',
                         linewidth=2)
        #Adds curved arcs to represent smooth lane transitions using the Arc patch.
        arc1 = Arc(xy=(transition_lines[0][0], transition_lines[0][1]), width=2*transition_lines[0][2],
                   height=2*transition_lines[0][2], angle=90, theta1=-180, theta2=-90, color='black', linewidth=2)
        fig.add_patch(arc1)
        arc2 = Arc(xy=(transition_lines[1][0], transition_lines[1][1]), width=2*transition_lines[1][2],
                   height=2*transition_lines[1][2], angle=90, theta1=-90, theta2=0, color='black', linewidth=2)
        fig.add_patch(arc2)
        arc3 = Arc(xy=(transition_lines[2][0], transition_lines[2][1]), width=2*transition_lines[2][2],
                   height=2*transition_lines[2][2], angle=90, theta1=90, theta2=180, color='black', linewidth=2)
        fig.add_patch(arc3)
        arc4 = Arc(xy=(transition_lines[3][0], transition_lines[3][1]), width=2*transition_lines[3][2],
                   height=2*transition_lines[3][2], angle=90, theta1=0, theta2=90, color='black', linewidth=2)
        fig.add_patch(arc4)

        return

    def create_lanes(self, fig):
        #Defines the lane structure of the map:
        #solid - rep lane boundaries
        solid_lines = [[0, 80, 0, 'h'], [0, 47.5, 14, 'h'], [66.5, 80, 14, 'h'], [50, 16.5, 70.5, 'v'],
                       [64, 16.5, 70.5, 'v'], [0, 47.5, 73, 'h'], [66.5, 80, 73, 'h'], [0, 80, 80, 'h']]
        #dotted - lane markers
        dotted_lines = [[0, 50, 3.5, 'h'], [0, 50, 7, 'h'], [0, 50, 10.5, 'h'], [64, 80, 3.5, 'h'],
                        [64, 80, 7, 'h'], [64, 80, 10.5, 'h'], [53.5, 14, 73, 'v'], [57, 14, 73, 'v'],
                        [60.5, 14, 73, 'v'], [0, 50, 76.5, 'h'], [64, 80, 76.5, 'h']]
        #Specifies curved arcs for lane transitions (e.g., at intersections).
        transition_lines = [[47.5, 16.5, 2.5], [47.5, 70.5, 2.5], [66.5, 16.5, 2.5], [66.5, 70.5, 2.5]]

        #Calls draw_lines to visualize the lanes and returns the lane data.
        self.draw_lines(fig, solid_lines, dotted_lines, transition_lines)

        return solid_lines, dotted_lines, transition_lines

    def draw_cycle(self, fig, obstacles):
        '''Draws obstacles on the map using Circle patches:
        Obstacles are classified by a type (obstacle[3]):
        Type 2: Yellow circles.
        Others: Red circles.'''
        for obstacle in obstacles:
            if obstacle[3] == 2:
                cir = Circle(xy=(obstacle[0], obstacle[1]), radius=obstacle[2], facecolor='yellow')
            else:
                cir = Circle(xy=(obstacle[0], obstacle[1]), radius=obstacle[2], facecolor='red')
            fig.add_patch(cir)

        return

    def get_obstacles(self, fig):
        #Returns a predefined list of obstacles: 
        #[posx,posy, radius, type, orientation]
        # obstacles = [[17, 1, 1.5, 2, 'h'], [30, 4, 2, 3, 'h'], [50.5, 28, 2, 2, 'v'], [64, 35, 1.5, 2, 'v']]
        obstacles = [[17, 1, 1.5, 2, 'h'], [30, 4, 2, 3, 'h'], [50.5, 28, 2, 2, 'v']] 
        self.draw_cycle(fig, obstacles)

        return obstacles

    def draw_vehicle(self, fig, vehicles, L, B):
        #Draws vehicles on the map using Rectangle patches: Vehicles can be horizontal ('h') or vertical ('v'):
        for vehicle in vehicles:
            if vehicle[2] == 'h':
                x1 = vehicle[0] - L / 2
                y1 = vehicle[1] - B / 2
                rec = Rectangle(xy=(x1, y1), width=L, height=B, color='blue')
                fig.add_patch(rec)
            else:
                x1 = vehicle[0] - B / 2
                y1 = vehicle[1] - L / 2
                rec = Rectangle(xy=(x1, y1), width=B, height=L, color='blue')
                fig.add_patch(rec)

        return

    def get_vehicles(self, fig, L, B):
        #gives a list of static and moving vehicles
        # vehicles = [[45, 8.5, 'h'], [5, 5.5, 'h'], [55, 40, 'v'], [58.5, 50, 'v'], [18, 12, 'h']]  #[x-coordinate, y-coordinate, orientation]
        # init_movingvehis = [[10, 85, 'h', 3], [10, 88, 'h', 3], [10, 90, 'h', 3]]  #[x-coordinate, y-coordinate, orientation, speed]
        # moving_vehicles = [[10, 85, 'h', 3], [10, 88, 'h', 3], [10, 90, 'h', 3]]

        vehicles = [[45, 8.5, 'h'], [5, 5.5, 'h']]
        init_movingvehis = [[55, 55, 'v', -3], [58.5, 50, 'v', 3], [15, 12, 'h', 3]]
        moving_vehicles = [[55, 55, 'v', -3], [58.5, 50, 'v', 3], [15, 12, 'h', 3]]

        self.draw_vehicle(fig, vehicles, L, B)

        return vehicles, init_movingvehis, moving_vehicles

    def get_startandtarget(self):
        # Define start and target positions for two vehicles
        starts = [[5, 2], [10, 5]]       # Start positions for both vehicles
        targets = [[78, 78], [70, 60]]   # Target positions for both vehicles

        # Plot the start and target positions
        for start, target in zip(starts, targets):
            plt.plot(start[0], start[1], '*', color='purple', markersize=10)  # Start points
            plt.plot(target[0], target[1], 'o', color='purple', markersize=10)  # Target points

        return starts, targets

        
    def final_draw(self, fig, front_points_list, paths_list, init_movingvehis, moving_vehicles, L, B):
        colors = ['red', 'blue']  # Use distinct colors for each vehicle

        # Loop through each vehicle's trajectory and front points
        for i in range(len(front_points_list)):
            front_x, front_y, path_x, path_y = [], [], [], []

            # Process front points
            for point in front_points_list[i]:
                front_x.append(point[0])
                front_y.append(point[1])
            plt.plot(front_x, front_y, 'o', color=colors[i], markersize=2, label=f"Vehicle {i+1} Front Points")

            # Process paths
            for path in paths_list[i]:
                path_x.append(path[0])
                path_y.append(path[1])
            plt.plot(path_x, path_y, color=colors[i], linewidth=2, label=f"Vehicle {i+1} Path")

        # Plot moving vehicles
        for v1, v2 in zip(init_movingvehis, moving_vehicles):
            if v1[2] == 'h':   # Horizontal vehicle
                x1 = v1[0] - L / 2  # Bottom-left x-coordinate
                y1 = v1[1] - B / 2  # Bottom-left y-coordinate
                h = v2[0] + L / 2 - x1
                rec = Rectangle(xy=(x1, y1), width=h, height=B, color='green', alpha=0.3)
                fig.add_patch(rec)  # Add to the plot
            else:  # Vertical vehicle
                x1 = v1[0] - B / 2
                y1 = v1[1] - L / 2
                h = v2[1] + L / 2 - y1   # Distance traveled vertically
                rec = Rectangle(xy=(x1, y1), width=B, height=h, color='green', alpha=0.3)
                fig.add_patch(rec)

        # Draw current positions of moving vehicles
        for vehicle in moving_vehicles:
            if vehicle[2] == 'h':
                x1 = vehicle[0] - L / 2
                y1 = vehicle[1] - B / 2
                rec = Rectangle(xy=(x1, y1), width=L, height=B, color='green')
                fig.add_patch(rec)
            else:
                x1 = vehicle[0] - B / 2
                y1 = vehicle[1] - L / 2
                rec = Rectangle(xy=(x1, y1), width=B, height=L, color='green')
                fig.add_patch(rec)

        plt.legend()
        return
