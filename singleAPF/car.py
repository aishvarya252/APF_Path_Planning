import math
import matplotlib.pyplot as plt


class CAR:

    def __init__(self, L, B, l, velocity):
        self.L = L
        self.B = B
        self.l = l
        self.v = velocity
        self.dif_distances = []

    def model(self, current_position, fi, si, del_t=0):
        '''This method calculates the vehicle's motion based on the kinematic bicycle model. It predicts the rear and front positions of the vehicle as it moves.'''
        w = self.v * math.tan(si) / self.l  #angular vel-Determines how fast the vehicle is turning based on its steering angle.
        vf = self.v / math.cos(si)          #front vel-Calculates the velocity of the front of the vehicle.

        rear_v = [self.v * math.cos(fi), self.v * math.sin(fi), w]  
        rear_p = [current_position[0] + del_t * rear_v[0],          #Computes the new position of the rear of the vehicle
                  current_position[1] + del_t * rear_v[1]]
        '''current_position: Current position of the rear of the vehicle [x, y, orientation (fi)].
            fi: Current orientation of the vehicle (angle in radians).
            si: Steering angle of the front wheels (in radians).
            del_t: Time step for the simulation (default is 0).'''
        current_front = [current_position[0] + self.l * math.cos(fi),
                         current_position[1] + self.l * math.sin(fi)]
        front_v = [vf * math.cos(fi + si), vf * math.sin(fi + si), w]
        front_p = [current_front[0] + del_t * front_v[0],
                   current_front[1] + del_t * front_v[1]]

        new_fi = fi + del_t * w

        return rear_v, rear_p, front_v, front_p, new_fi, vf

    def difference(self, rear_p, front_p, si, del_t=0):
        #calculates the difference in distance between two approaches:
        dif_distance = math.sqrt(self.l ** 2 + (del_t * self.v * math.tan(si)) ** 2) - self.l
        dif_distance2 = math.sqrt((front_p[0] - rear_p[0]) ** 2 + (front_p[1] - rear_p[1]) ** 2) - self.l
        self.dif_distances.append(dif_distance)

        return dif_distance, dif_distance2

    def draw_car(self, rear_p, front_p):
        #This method visualizes the vehicle as a line connecting the rear and front positions:
        draw_x = [rear_p[0], front_p[0]]
        draw_y = [rear_p[1], front_p[1]]
        #Uses matplotlib to plot an orange line between the rear and front positions
        plt.plot(draw_x, draw_y, c='orange', linewidth=3)
        #Returns the two distances for debugging or validation.
        return

    def draw_movingcar(self, moving_vehicles, del_t=0):
        #visualizes moving vehicles and updates their positions over time.
        '''moving_vehicles: List of moving vehicles with their current positions and velocities. Each entry is a list:
            [x, y, orientation, velocity]
            del_t: Time step for updating positions.'''
        #Checks if there are any moving vehicles in the list. If not, it returns the empty list and None for the plot elements.   
        if not moving_vehicles:
            return moving_vehicles, None, None, None
        
        
        new_moving_vehicles, p1, p2, = [], [], []
        #Each vehicle is represented as a green line.
        x11 = moving_vehicles[0][0]
        y11 = moving_vehicles[0][1] - self.l / 2
        x12 = moving_vehicles[0][0]
        y12 = moving_vehicles[0][1] + self.l / 2
        draw_x1 = [x11, x12]
        draw_y1 = [y11, y12]
        p1, = plt.plot(draw_x1, draw_y1, c='green', linewidth=10)

        x21 = moving_vehicles[1][0]
        y21 = moving_vehicles[1][1] - self.l / 2
        x22 = moving_vehicles[1][0]
        y22 = moving_vehicles[1][1] + self.l / 2
        draw_x2 = [x21, x22]
        draw_y2 = [y21, y22]
        p2, = plt.plot(draw_x2, draw_y2, c='green', linewidth=10)

        x31 = moving_vehicles[2][0] - self.l / 2
        y31 = moving_vehicles[2][1]
        x32 = moving_vehicles[2][0] + self.l / 2
        y32 = moving_vehicles[2][1]
        draw_x3 = [x31, x32]
        draw_y3 = [y31, y32]
        p3, = plt.plot(draw_x3, draw_y3, c='green', linewidth=10)

        coop_x1 = moving_vehicles[0][0]
        coop_y1 = moving_vehicles[0][1] + moving_vehicles[0][3] * del_t
        coop_x2 = moving_vehicles[1][0]
        coop_y2 = moving_vehicles[1][1] + moving_vehicles[1][3] * del_t
        coop_x3 = moving_vehicles[2][0] + moving_vehicles[2][3] * del_t
        coop_y3 = moving_vehicles[2][1]
        #Updates the moving_vehicles list with the new positions:
        new_moving_vehicles.append([coop_x1, coop_y1, 'v', moving_vehicles[0][3]])
        new_moving_vehicles.append([coop_x2, coop_y2, 'v', moving_vehicles[1][3]])
        new_moving_vehicles.append([coop_x3, coop_y3, 'h', moving_vehicles[2][3]])

        moving_vehicles.clear()
        moving_vehicles = new_moving_vehicles

        return moving_vehicles, p1, p2, p3,
