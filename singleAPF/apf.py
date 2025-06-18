import math
import matplotlib.pyplot as plt
from car import *
from supplement import *
from vehicle_geometry import compute_corners
from polygon_dist import min_distance_between_polygons

class APF:

    def __init__(self, k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, target_area, F_e, del_t, max_iters, solid_lines, transition_lines, obstacles, vehicles, L, B, car, num_vehicles,vehicle_ids,boundaries_raw, obstacles_raw):
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
        self.vehicle_ids = vehicle_ids
        self.boundary_polygons = boundaries_raw
        self.obstacle_polygons = obstacles_raw


        # State variables (now vectors of vectors for multiple vehicles)
        self.rear_velocities = [[] for _ in range(num_vehicles)]   # Rear velocities for 2 vehicles
        self.paths = [[] for _ in range(num_vehicles)]            # Trajectories (rear positions) for 2 vehicles
        self.directions = [[] for _ in range(num_vehicles)]       # Directions (orientations) for 2 vehicles
        self.front_velocities = [[] for _ in range(num_vehicles)] # Front velocities for 2 vehicles
        self.front_points = [[] for _ in range(num_vehicles)]     # Front positions for 2 vehicles
        self.steerings = [[] for _ in range(num_vehicles)]        # Steering angles for 2 vehicles
        self.vehicle_corners = [[] for _ in range(num_vehicles)]
        self.min_distances = [[] for _ in range(num_vehicles)]  # Per vehicle per iteration
        self.closest_point_pairs = [[] for _ in range(num_vehicles)]  # [(pt_on_vehicle, pt_on_polygon)]

        # Forces (vectors of vectors for each vehicle)
        self.F_atts = [[] for _ in range(num_vehicles)]           # Attraction forces
        self.F_lanes = [[] for _ in range(num_vehicles)]          # Lane-keeping forces
        self.F_obss = [[] for _ in range(num_vehicles)]           # Obstacle repulsion forces
        self.F_vehs = [[] for _ in range(num_vehicles)]           # Vehicle repulsion forces
        self.F_totals = [[] for _ in range(num_vehicles)]         # Total forces
        self.debug_logs = []  # Store logs per iteration per vehicle



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

        '''        print("\n🟦 [keeplane FUNCTION START] 🟦")
        print(f"  Input Front Position: {front_p} (type: {type(front_p)})")
        print(f"  Horizontal Solid Lines: {supply.h_sollines}")
        print(f"  Vertical Solid Lines  : {supply.v_sollines}")'''
        
        #Identifies nearby solid lane lines using a SUPPLY class.
        use_sollinesx, use_sollinesy = supply.useful_lines(front_p)

        '''Computes corrective forces:
        Horizontal (lane_x), Vertical (lane_y), or both (lane_xy).
        Handles intersections by using a generic lane correction force (lane).'''
        
        if use_sollinesx != [] and use_sollinesy != []:
            F_lane = supply.lane_xy(front_p, use_sollinesx, use_sollinesy)
            print("  ➤ Using Combined Lane Correction (lane_xy)")
        elif use_sollinesx != [] and use_sollinesy == []:
            F_lane = supply.lane_x(front_p, use_sollinesx)
            print("  ➤ Using Horizontal Lane Correction (lane_x)")
        elif use_sollinesy != [] and use_sollinesx == []:
            F_lane = supply.lane_y(front_p, use_sollinesy)
            print("  ➤ Using Vertical Lane Correction (lane_y)")
        else:
            F_lane = supply.lane(front_p)
            print("  ➤ Using Default Lane Correction (lane)")
            
        print(f"  Output Front Pos       : {front_p}")
        print(f"  Computed Lane Force    : {F_lane}")
        
        # Ensure F_lane is always a flat list [Fx, Fy]
        if isinstance(F_lane, list) and len(F_lane) > 0 and isinstance(F_lane[0], list):
            F_lane = F_lane[0]  # Extract inner list if it's nested
        elif not isinstance(F_lane, list):  
            F_lane = [F_lane]  # Ensure it remains a list

        print("🟦 [keeplane FUNCTION END] 🟦\n")
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
            #elif r_obs <= 0:
                #print("APF is failed, because the vehicle moves into the safe area.")
                #exit()
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

        return F_obs
    
    def repulsion_force_vector(self, alpha, e, k_rep):
        """
        Compute repulsion force vector from point e (static) on point alpha (vehicle).

        Args:
            alpha: (x, y) vehicle closest point
            e: (x, y) static polygon closest point
            k_rep: repulsion coefficient

        Returns:
            Force vector (Fx, Fy) pushing vehicle point alpha away from e
        """
        k_rep = 500
        dx = alpha[0] - e[0]
        dy = alpha[1] - e[1]

        dist = math.sqrt(dx*dx + dy*dy)

        if dist == 0:
            return (0.0, 0.0)  # avoid division by zero

        force_magnitude = k_rep / (dist**3)
        Fx = force_magnitude * dx
        Fy = force_magnitude * dy
        return Fx, Fy


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

        return F_veh
    
    def pathplanning(self, rear_p_list, front_p_list, fi_list, targets, iters_list, moving_vehicles):
        # Create SUPPLY and EMERGENCY objects for lane and obstacle handling
        supply = SUPPLY(self.k_replane, self.k_lane, self.lw, self.h_sollines, self.v_sollines, self.t_lines)
        emergency = EMERGENCY(self.F_e, self.h_sollines, self.v_sollines, self.lw)
        
        vehicles_reached_target = [False] * len(rear_p_list)  # Track vehicles that have reached their target
        
        print("\n🚀 [APF PATH PLANNING STARTED] 🚀")
        print(f"Number of Vehicles: {len(rear_p_list)}\n")

        forces = []  # Store forces for all vehicles
        steering_angles = []  # Store steering angles for all vehicles
        updated_positions = []  # Store updated rear and front positions
        updated_orientations = []  # Store updated orientations
        updated_velocities = []  # Store updated velocities
        
        # Create a per-vehicle log list
        vehicle_logs = [[] for _ in range(len(rear_p_list))]
            
        # Loop through each vehicle for simultaneous path planning
        # **Step 1: Compute Forces for All Vehicles**
        for i in range(len(rear_p_list)):
            if vehicles_reached_target[i]:  # Skip if vehicle already reached its target
                forces.append(None)
                steering_angles.append(None)
                continue
            
            rear_p = rear_p_list[i]
            front_p = front_p_list[i]
            fi = fi_list[i]
            target = targets[i]
            iters_list[i] += 1  # Increment iteration count
            vehicle_id = self.vehicle_ids[i]
            
            # Calculate initial distance to the target
            distance = math.sqrt((target[0] - rear_p[0]) ** 2 + (target[1] - rear_p[1]) ** 2)
                
            log_str = []

            print("\n" + "="*60)
            print(f"🚗 [ITERATION START] Vehicle ID: {vehicle_id} | Index: {i} | Iteration: {iters_list[i]}")
            print(f"🔹 Rear Position     : {rear_p}")
            print(f"🔹 Front Position    : {front_p}")
            print(f"🔹 Orientation (rad) : {fi:.4f}")
            print(f"🔹 Target Position   : {target}")
            print(f"🔹 Distance to Target: {distance:.2f}")
            print("="*60)
            print(f"  🎯 Vehicle ID {vehicle_id} pursuing: {target}")
            
            vehicle_logs[i].append("=" * 60)
            vehicle_logs[i].append(f"🚗 [ITERATION START] Vehicle ID: {self.vehicle_ids[i]} | Index: {i} | Iteration: {iters_list[i]}")
            vehicle_logs[i].append(f"🔹 Rear Position     : {rear_p}")
            vehicle_logs[i].append(f"🔹 Front Position    : {front_p}")
            vehicle_logs[i].append(f"🔹 Orientation (rad) : {fi:.4f}")
            vehicle_logs[i].append(f"🔹 Target Position   : {target}")
            vehicle_logs[i].append(f"🔹 Distance to Target: {distance:.2f}")
            vehicle_logs[i].append("=" * 60)
            vehicle_logs[i].append(f"  🎯 Vehicle ID {self.vehicle_ids[i]} pursuing: {target}")

                
            if distance <= self.target_area:
                print(f"Vehicle {vehicle_id} is within target area.")
                si = 0  # keep orientation steady
                F_total = [0, 0]  # no further attraction
                forces.append(F_total)
                steering_angles.append(si)
                continue
                
            # Calculate forces
            F_att = self.attraction(front_p, target)
            #print(f"  → [Attraction Force] F_att = {F_att}")
                
            #Calculate lane keeping forces
            F_lane, use_sollinesx, use_sollinesy = self.keeplane(front_p, supply)
            #print(f"  → [Lane Keeping Force] F_lane = {F_lane}")
                
                #calculate obs repulsion force
                #F_obs = self.repulsion(front_p, emergency, use_sollinesx, use_sollinesy)
                #print(f"  → [Obstacle Repulsion Force] F_obs = {F_obs}")
                
                #calculate vehicle repulsion force
            F_veh = self.leavecar(front_p, moving_vehicles, emergency, use_sollinesx, use_sollinesy)
            #print(f"  → [Vehicle Repulsion Force] F_veh = {F_veh}")
                
            if not self.closest_point_pairs[i]:
                # Compute closest point just-in-time
                vehicle_poly = compute_corners(rear_p[0], rear_p[1], fi, self.L, self.B)
                min_dist = float('inf')
                for static_poly in self.boundary_polygons + self.obstacle_polygons:
                    dist, pt_vehicle, pt_static = min_distance_between_polygons(vehicle_poly, static_poly)
                    if dist < min_dist:
                        alpha, e = pt_vehicle, pt_static
                            
            else:
                alpha, e = self.closest_point_pairs[i][-1]
            Fx_rep, Fy_rep = self.repulsion_force_vector(alpha, e, self.k_rep)
            #print(f"  🚫 Boundary Repulsion Force: ({Fx_rep:.2f}, {Fy_rep:.2f})")
                
                # Visualize the repulsion force from closest point on vehicle polygon
                #plt.quiver(alpha[0], alpha[1], Fx_rep, Fy_rep, color='purple', angles='xy', scale_units='xy', scale=20, width=0.003)

            F_total = [F_att[0] + F_lane[0] + F_veh[0], 
                        F_att[1] + F_lane[1] + F_veh[1]]
                
            # Add to total force
            F_total[0] += Fx_rep
            F_total[1] += Fy_rep
                           
            # Calculate force direction and steering angle, Converts force components into an angle.
            F_direction = math.atan2(F_total[1], F_total[0])

            #Limit Steering Angle (si), If the difference between the force direction and the vehicle’s current orientation fi exceeds ±40°, it is capped.
            if F_direction - fi > 40 / 180 * math.pi:
                si = 40 / 180 * math.pi
            elif F_direction - fi < -40 / 180 * math.pi:
                si = -40 / 180 * math.pi
            else:
                si = F_direction - fi
            print(f"  ⇒ [Steering Angle] {math.degrees(si):.2f}°")
                
            forces.append(F_total)
            steering_angles.append(si)
            
            print(f"  → [Attraction Force] F_att = {F_att}")
            print(f"  → [Lane Keeping Force] F_lane = {F_lane}")
            print(f"  → [Vehicle Repulsion Force] F_veh = {F_veh}")
            print(f"  🚫 Boundary Repulsion Force: ({Fx_rep:.2f}, {Fy_rep:.2f})")
            print(f"  ⇒ [Total Force] F_total = {F_total}")
            print(f"  ⇒ [Force Direction] {math.degrees(F_direction):.2f}°")
            print(f"  ⇒ [Steering Angle] {math.degrees(si):.2f}°")
            print(f"✅ [ITERATION END] Vehicle ID: {vehicle_id} | Iteration {iters_list[i]}")
            print("-" * 60)

            vehicle_logs[i].append(f"  → [Attraction Force] F_att = {F_att}")
            vehicle_logs[i].append(f"  → [Lane Keeping Force] F_lane = {F_lane}")
            vehicle_logs[i].append(f"  → [Vehicle Repulsion Force] F_veh = {F_veh}")
            vehicle_logs[i].append(f"  🚫 Boundary Repulsion Force: ({Fx_rep:.2f}, {Fy_rep:.2f})")
            vehicle_logs[i].append(f"  ⇒ [Total Force] F_total = {F_total}")
            vehicle_logs[i].append(f"  ⇒ [Force Direction] {math.degrees(F_direction):.2f}°")
            vehicle_logs[i].append(f"  ⇒ [Steering Angle] {math.degrees(si):.2f}°")
            vehicle_logs[i].append(f"✅ [ITERATION END] Vehicle ID: {self.vehicle_ids[i]} | Iteration {iters_list[i]}")
            vehicle_logs[i].append("-" * 60)
            
            self.debug_logs.append({
                "vehicle_id": self.vehicle_ids[i],
                "iteration": iters_list[i],
                "log": "\n".join(log_str)
            })

        # **Step 2: Update Positions for All Vehicles**
        for i in range(len(rear_p_list)):
            if vehicles_reached_target[i]:  # Skip already reached vehicles
                continue
                
                # Retrieve computed force and steering for this vehicle
            F_total = forces[i]
            si = steering_angles[i]
                
                # Update vehicle state
            rear_v, rear_p, front_v, front_p, fi, vf = self.car.model(rear_p_list[i], fi_list[i], si, self.del_t)

                # Checks for inconsistencies in position updates
            dif_distance, dif_distance2 = self.car.difference(rear_p, front_p, si, self.del_t)
                
            print("\n🔄 [Vehicle Update]")
            print(f"  Rear Velocity : {rear_v}")
            print(f"  New Rear Pos  : {rear_p}")
            print(f"  New Front Pos : {front_p}")
            print(f"  New Orientation: {fi:.4f} rad")
            print(f"  Velocity (Front Axle) = {vf}")
            print(f"  Rear-Front Diff Check = {dif_distance:.4f}, Formula Diff = {dif_distance - dif_distance2:.4f}")

            vehicle_logs[i].append("🔄 [Vehicle Update]")
            vehicle_logs[i].append(f"  Rear Velocity : {rear_v}")
            vehicle_logs[i].append(f"  New Rear Pos  : {rear_p}")
            vehicle_logs[i].append(f"  New Front Pos : {front_p}")
            vehicle_logs[i].append(f"  New Orientation: {fi:.4f} rad")
            vehicle_logs[i].append(f"  Velocity (Front Axle) = {vf}")
            vehicle_logs[i].append(f"  Rear-Front Diff Check = {dif_distance:.4f}, Formula Diff = {dif_distance - dif_distance2:.4f}")

                
            # **Update the vehicle lists**
            rear_p_list[i] = rear_p
            front_p_list[i] = front_p
            fi_list[i] = fi
                
            # Compute corners and store
            corners = compute_corners(rear_p[0], rear_p[1], fi, self.L, self.B)
            self.vehicle_corners[i].append(corners)
                    
            vehicle_poly = corners  # Already in C1–C4 format
            min_dist = float('inf')
            closest_vehicle_pt = None
            closest_static_pt = None

            for static_poly in self.boundary_polygons + self.obstacle_polygons:
                dist, pt_vehicle, pt_static = min_distance_between_polygons(vehicle_poly, static_poly)
                if dist < min_dist:
                    min_dist = dist
                    closest_vehicle_pt = pt_vehicle
                    closest_static_pt = pt_static

            self.min_distances[i].append(min_dist)
            self.closest_point_pairs[i].append((closest_vehicle_pt, closest_static_pt))
            
            print(f"🧩 Vehicle {self.vehicle_ids[i]} Corners (C1 to C4):")
            for j, corner in enumerate(corners, 1):
                print(f"  C{j}: ({corner[0]:.2f}, {corner[1]:.2f})")

            print(f"📏 Min distance to map (vehicle {vehicle_id}): {min_dist:.2f}")
            print(f"  🔸 Closest point on vehicle: {closest_vehicle_pt}")
            print(f"  🔸 Closest point on polygon: {closest_static_pt}")

            vehicle_logs[i].append(f"🧩 Vehicle {self.vehicle_ids[i]} Corners (C1 to C4):")
            for j, corner in enumerate(corners, 1):
                vehicle_logs[i].append(f"  C{j}: ({corner[0]:.2f}, {corner[1]:.2f})")
            vehicle_logs[i].append(f"📏 Min distance to map (vehicle {self.vehicle_ids[i]}): {min_dist:.2f}")
            vehicle_logs[i].append(f"  🔸 Closest point on vehicle: {closest_vehicle_pt}")
            vehicle_logs[i].append(f"  🔸 Closest point on polygon: {closest_static_pt}")

            # Store updated values for simultaneous update
            updated_positions.append((rear_p, front_p))
            updated_orientations.append(fi)
            updated_velocities.append((rear_v, front_v))
                
            # Store historical values for analysis
            self.rear_velocities[i].append(rear_v)
            self.paths[i].append([rear_p[0], rear_p[1]])
            self.directions[i].append(fi)
            self.front_velocities[i].append(vf)
            self.front_points[i].append([front_p[0], front_p[1]])
            self.steerings[i].append(si)
                
        draw_index = 0
        for i in range(len(rear_p_list)):
            if vehicles_reached_target[i]:
                continue
            self.car.draw_car(updated_positions[draw_index][0], updated_positions[draw_index][1])
            draw_index += 1
            
        for i in range(len(rear_p_list)):
            self.debug_logs.append({
                "vehicle_id": self.vehicle_ids[i],
                "iteration": iters_list[i],
                "log": "\n".join(vehicle_logs[i])
            })

        # **Step 4: Update Moving Vehicles and Refresh Frame**
        # Only call draw_movingcar if there are any moving vehicles
        if moving_vehicles:
            moving_vehicles, p1, p2, p3 = self.car.draw_movingcar(moving_vehicles, self.del_t)
            plt.pause(0.1)
            if p1: p1.remove()
            if p2: p2.remove()
            if p3: p3.remove()
        else:
            plt.pause(0.1)  # just pause to show vehicle motion

        return rear_p_list, front_p_list, fi_list, iters_list, moving_vehicles
    
    def save_debug_logs_to_txt(self, filepath):
        with open(filepath, "a", encoding="utf-8") as f:  # <-- Add encoding="utf-8"
            for entry in self.debug_logs:
                f.write(f"Vehicle ID: {entry['vehicle_id']} | Iteration: {entry['iteration']}\n")
                f.write(entry['log'])
                f.write("\n" + "="*80 + "\n")
        self.debug_logs.clear()