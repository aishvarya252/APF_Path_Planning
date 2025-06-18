import math
import matplotlib.pyplot as plt
from car import *
from supplement import *


class APF:

    def __init__(self, k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, target_area, F_e, del_t, max_iters, solid_lines, transition_lines, obstacles, vehicles, L, B, car):
        # Force parameters
        self.k_att = k_att           # Strength of attraction towards the target
        self.k_rep = k_rep           # Strength of repulsion from obstacles
        self.r_apf = r_apf           # Radius of influence for obstacles
        self.k_replane = k_replane
        self.k_lane = k_lane         # Strength of lane-keeping forces
        self.k_car = k_car           # Strength of repulsion from other vehicles

        # Map and vehicle parameters
        self.lw = lane_width         # Lane width
        self.target_area = target_area   # Threshold distance to consider target reached
        self.F_e = F_e               # Emergency force constant
        self.del_t = del_t           # Time step
        self.max_iters = max_iters   # Maximum iterations for path planning
        self.h_sollines, self.v_sollines = self.solid(solid_lines)  # Organize solid lines
        self.t_lines = transition_lines  # Transition lines (curved lanes)
        self.obstacles = obstacles       # Obstacles on the map
        self.vehicles = vehicles         # Static vehicles on the map
        self.L = L                       # Vehicle length
        self.B = B                       # Vehicle width
        self.car_Lmin = 2 * L            # Minimum longitudinal spacing for safety
        self.car_Bmin = B                # Minimum lateral spacing for safety
        self.car = car                   # Vehicle model

        # State variables (now vectors of vectors for multiple vehicles)
        self.rear_velocities = [[] for _ in range(2)]   # Rear velocities for 2 vehicles
        self.paths = [[] for _ in range(2)]            # Trajectories (rear positions) for 2 vehicles
        self.directions = [[] for _ in range(2)]       # Directions (orientations) for 2 vehicles
        self.front_velocities = [[] for _ in range(2)] # Front velocities for 2 vehicles
        self.front_points = [[] for _ in range(2)]     # Front positions for 2 vehicles
        self.steerings = [[] for _ in range(2)]        # Steering angles for 2 vehicles

        # Forces (vectors of vectors for each vehicle)
        self.F_atts = [[] for _ in range(2)]           # Attraction forces
        self.F_lanes = [[] for _ in range(2)]          # Lane-keeping forces
        self.F_obss = [[] for _ in range(2)]           # Obstacle repulsion forces
        self.F_vehs = [[] for _ in range(2)]           # Vehicle repulsion forces
        self.F_totals = [[] for _ in range(2)]         # Total forces

    def solid(self, solid_lines):
        #helper function organizes solid lines:
        h_sollines, v_sollines = [], []

        for solid_line in solid_lines:
            if solid_line[3] == 'h':
                h_sollines.append(solid_line)
            else:
                v_sollines.append(solid_line)

        return h_sollines, v_sollines

    def attraction(self, front_p, target):
        #Uses the position of the vehicle's front (front_p) and the target point.
        force_att = self.k_att
        #Direction angle (theta_att) using atan2.
        theta_att = math.atan2(target[1] - front_p[1], target[0] - front_p[0])
        #Force components (F_attx, F_atty) using trigonometric functions.
        F_attx = force_att * math.cos(theta_att)
        F_atty = force_att * math.sin(theta_att)
        F_att = [F_attx, F_atty]

        return F_att

    def keeplane(self, front_p, supply):

        print(f"Debugging keeplane() - front_p: {front_p}, type: {type(front_p)}")

        print("Debugging keeplane() - supply.h_sollines:", supply.h_sollines)
        print("Debugging keeplane() - supply.v_sollines:", supply.v_sollines)
        
        #Identifies nearby solid lane lines using a SUPPLY class.
        use_sollinesx, use_sollinesy = supply.useful_lines(front_p)

        '''Computes corrective forces:
Horizontal (lane_x), Vertical (lane_y), or both (lane_xy).
Handles intersections by using a generic lane correction force (lane).'''
        if use_sollinesx != [] and use_sollinesy != []:
            F_lane = supply.lane_xy(front_p, use_sollinesx, use_sollinesy)
        elif use_sollinesx != [] and use_sollinesy == []:
            F_lane = supply.lane_x(front_p, use_sollinesx)
        elif use_sollinesy != [] and use_sollinesx == []:
            F_lane = supply.lane_y(front_p, use_sollinesy)
        else:
            F_lane = supply.lane(front_p)
            
        print("Front_pos in keeplane after useful_line: ", front_p)
        print("Debugging keeplane() - F_lane:", F_lane)
        
        # Ensure F_lane is always a flat list [Fx, Fy]
        if isinstance(F_lane, list) and len(F_lane) > 0 and isinstance(F_lane[0], list):
            F_lane = F_lane[0]  # Extract inner list if it's nested
        elif not isinstance(F_lane, list):  
            F_lane = [F_lane]  # Ensure it remains a list

        #Returns the force vector and relevant lane lines
        return F_lane, use_sollinesx, use_sollinesy

    def repulsion(self, front_p, emergency, use_sollinesx, use_sollinesy):
        #repulsion from obstacles:
        F_obsx, F_obsy = 0, 0

        '''For each obstacle:
Calculate the distance (r_obs) between the front of the vehicle and the obstacle.
Compare r_obs with the safety radius (r_min).
If r_obs is too small, calculate a repulsion force using force_rep
Special handling:
If the vehicle enters a "local minimum," emergency forces are applied using the EMERGENCY class.'''
        for obstacle in self.obstacles:
            r_obs = math.sqrt((front_p[0] - obstacle[0]) ** 2 + (front_p[1] - obstacle[1]) ** 2) - 1.5 * obstacle[2]
            r_min = obstacle[3] * self.r_apf
            obs_Le = obstacle[3] * obstacle[2]  # Obstacle length scaling.
            obs_Be = 0.5 * obstacle[3] * self.B  #Obstacle breadth scaling (half of obstacle length factor B)

            if obstacle[4] == 'h':
                obs_L = abs(front_p[0] - obstacle[0])
                obs_B = abs(front_p[1] - obstacle[1])
            else:
                obs_L = abs(front_p[1] - obstacle[1])
                obs_B = abs(front_p[0] - obstacle[0])

            if r_obs > r_min:
                continue
            elif r_obs <= 0:
                print("APF is failed, because the vehicle moves into the safe area.")
                exit()
            elif obs_L <= obs_Le and obs_B <= obs_Be:
                # The vehicle is stuck inside the obstacle’s influence zone.Calls emergency.obstacle_force() to compute emergency forces (Fe_obsx, Fe_obsy).
                print("The vehicle meets the local minimum.")
                Fe_obsx, Fe_obsy = emergency.obstacle_force(obstacle, use_sollinesx, use_sollinesy)
                force_rep = 2 * self.k_rep * obstacle[3] * (1 / r_obs - 1 / r_min) / r_obs ** 2
                theta_rep = math.atan2(front_p[1] - obstacle[1], front_p[0] - obstacle[0])
                F_obsx += force_rep * math.cos(theta_rep) + Fe_obsx
                F_obsy += force_rep * math.sin(theta_rep) + Fe_obsy
            else:
                #apply normal rep forces
                force_rep = 2 * self.k_rep * obstacle[3] * (1 / r_obs - 1 / r_min) / r_obs ** 2
                theta_rep = math.atan2(front_p[1] - obstacle[1], front_p[0] - obstacle[0])
                F_obsx += force_rep * math.cos(theta_rep)
                F_obsy += force_rep * math.sin(theta_rep)

        F_obs = [F_obsx, F_obsy]
        print("obs Repulsion force for vehicl: ",F_obs)

        return F_obs

    def leavecar(self, front_p, moving_vehicles, emergency, use_sollinesx, use_sollinesy):
        
        '''Checks distances between the front of the vehicle and surrounding vehicles.
Applies repulsion forces similar to obstacles, but considers vehicle-specific geometry (car_L, car_B).
Uses exponential decay functions for smooth force gradients.'''
        F_vehx, F_vehy = 0, 0
        car_Le = self.L
        car_Be = 0.5 * self.B
        all_vehicles = self.vehicles + moving_vehicles  #Combines stationary and moving vehicles into a single list.

        for vehicle in all_vehicles:
            #Determines how far the vehicle is along (car_L) and across (car_B) the other vehicle.
            if vehicle[2] == 'h':
                car_L = abs(front_p[0] - vehicle[0])
                car_B = abs(front_p[1] - vehicle[1])
            else:
                car_L = abs(front_p[1] - vehicle[1])
                car_B = abs(front_p[0] - vehicle[0])

            if car_L > self.car_Lmin or car_B > self.car_Bmin:
                #vehicle is outside the influence zone 
                continue
            #elif car_L == 0 and car_B == 0:
                #print("APF is failed, because the vehicle vehicle crashes a car.")
                #exit()
            elif car_L <= car_Le and car_B <= car_Be:
                #vehicle is stuck inside another vehicle’s influence zone.
                print("The vehicle meets the local minimum.")
                Fe_vehx, Fe_vehy = emergency.vehicle_force(vehicle, use_sollinesx, use_sollinesy)
                if vehicle[2] == 'h':
                    F_vehx += (front_p[0] - vehicle[0]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L -
                        (front_p[1] - vehicle[1]) ** 2 / 2 / self.B) + Fe_vehx
                    F_vehy += (front_p[1] - vehicle[1]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L -
                        (front_p[1] - vehicle[1]) ** 2 / 2 / self.B) + Fe_vehy
                else:
                    F_vehx += (front_p[0] - vehicle[0]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B -
                        (front_p[1] - vehicle[1]) ** 2 / 6 / self.L) + Fe_vehx
                    F_vehy += (front_p[1] - vehicle[1]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B -
                        (front_p[1] - vehicle[1]) ** 2 / 6 / self.L) + Fe_vehy
            else:
                if vehicle[2] == 'h':
                    F_vehx += (front_p[0] - vehicle[0]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L - (front_p[1] - vehicle[1]) ** 2 / 2 / self.B)
                    F_vehy += (front_p[1] - vehicle[1]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 6 / self.L - (front_p[1] - vehicle[1]) ** 2 / 2 / self.B)
                else:
                    F_vehx += (front_p[0] - vehicle[0]) / self.B * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B - (front_p[1] - vehicle[1]) ** 2 / 6 / self.L)
                    F_vehy += (front_p[1] - vehicle[1]) / 3 / self.L * self.k_car * math.exp(
                        -(front_p[0] - vehicle[0]) ** 2 / 2 / self.B - (front_p[1] - vehicle[1]) ** 2 / 6 / self.L)

        F_veh = [F_vehx, F_vehy]
        print("Veh Repulsion force for vehicle: ",F_veh)

        return F_veh
        
    def pathplanning(self, rear_p_list, front_p_list, fi_list, targets, iters_list, moving_vehicles):
        # Create SUPPLY and EMERGENCY objects for lane and obstacle handling
        supply = SUPPLY(self.k_replane, self.k_lane, self.lw, self.h_sollines, self.v_sollines, self.t_lines)
        emergency = EMERGENCY(self.F_e, self.h_sollines, self.v_sollines, self.lw)

        # Loop through each vehicle for simultaneous path planning
        for i in range(len(rear_p_list)):
            rear_p = rear_p_list[i]
            front_p = front_p_list[i]
            fi = fi_list[i]
            target = targets[i]
            iters = iters_list[i]
            print("IN PATHPLANNING: FOR VEHICLE" ,i, ":", rear_p, front_p, fi, target)

            # Calculate initial distance to the target
            distance = math.sqrt((target[0] - rear_p[0]) ** 2 + (target[1] - rear_p[1]) ** 2)

            while distance > self.target_area and iters <= self.max_iters:
                iters += 1
                print(f"\n----------\nVehicle {i + 1}: Iteration {iters}")

                # Calculate forces
                F_att = self.attraction(front_p, target)
                self.F_atts[i].append(F_att)
                
                #Calculate lane keeping forces
                F_lane, use_sollinesx, use_sollinesy = self.keeplane(front_p, supply)
                self.F_lanes[i].append(F_lane)
                
                #calculate obs repulsion force
                F_obs = self.repulsion(front_p, emergency, use_sollinesx, use_sollinesy)
                self.F_obss[i].append(F_obs)
                
                #calculate vehicle repulsion force
                F_veh = self.leavecar(front_p, moving_vehicles, emergency, use_sollinesx, use_sollinesy)
                self.F_vehs[i].append(F_veh)

                # Total force
                F_total = [F_att[0] + F_lane[0] + F_obs[0] + F_veh[0], F_att[1] + F_lane[1] + F_obs[1] + F_veh[1]]
                self.F_totals[i].append(F_total)

                # Calculate force direction and steering angle,Converts force components into an angle.
                F_direction = math.atan2(F_total[1], F_total[0])
                print(f"Vehicle {i + 1}: Total force:", F_total)
                print(f"Vehicle {i + 1}: Force direction:", int(F_direction / math.pi * 180))

                '''Limit Steering Angle (si), If the difference between the force direction and the vehicle’s current orientation fi exceeds ±40°, it is capped.'''
                if F_direction - fi > 40 / 180 * math.pi:
                    si = 40 / 180 * math.pi
                elif F_direction - fi < -40 / 180 * math.pi:
                    si = -40 / 180 * math.pi
                else:
                    si = F_direction - fi
                print(f"Vehicle {i + 1}: Steering angle:", int(si / math.pi * 180))

                # Update vehicle state
                rear_v, rear_p, front_v, front_p, fi, vf = self.car.model(rear_p, fi, si, self.del_t)
                print(f"Vehicle {i + 1}: Rear velocity: {rear_v}, Rear position: {rear_p}")

                # Checks for inconsistencies in position updates
                dif_distance, dif_distance2 = self.car.difference(rear_p, front_p, si, self.del_t)
                print(f"Vehicle {i + 1}: Difference between formula and subtract: {dif_distance - dif_distance2}")
                print(f"Vehicle {i + 1}: Distance difference: {dif_distance}")

                # Draw vehicle and moving cars
                self.car.draw_car(rear_p, front_p)
                moving_vehicles, p1, p2, p3 = self.car.draw_movingcar(moving_vehicles, self.del_t)
                plt.pause(0.1)
                p1.remove()
                p2.remove()
                p3.remove()

                # Append state updates for the current vehicle
                self.rear_velocities[i].append(rear_v)
                self.paths[i].append([rear_p[0], rear_p[1]])
                self.directions[i].append(fi)
                self.front_velocities[i].append(vf)
                self.front_points[i].append([front_p[0], front_p[1]])
                self.steerings[i].append(si)

                # Recalculate distance to target
                distance = math.sqrt((target[0] - rear_p[0]) ** 2 + (target[1] - rear_p[1]) ** 2)

            # Check if the vehicle reached its target
            if distance <= self.target_area and iters <= self.max_iters:
                print(f"\n----------\nVehicle {i + 1}: APF is successful!\n----------")
            else:
                print(f"\n----------\nVehicle {i + 1}: APF has failed!\n----------")

            # Update iteration count for the current vehicle
            iters_list[i] = iters

        return rear_p_list, front_p_list, fi_list, iters_list, moving_vehicles