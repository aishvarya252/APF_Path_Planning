import math
import matplotlib.pyplot as plt
import time
from assistant import *
from car import *
from apf import *

# main
if __name__ == "__main__":
    print("Creating a map:\n----------")
    length = 80
    width = 80
    del_block = 2

    map = MAP(length, width, del_block)
    fig = map.create_map()

    print("\nCreating lanes:\n----------")
    solid_lines, dotted_lines, transition_lines = map.create_lanes(fig)
    print("solid lines:", solid_lines)
    print("dotted lines:", dotted_lines)
    print("transition lines:", transition_lines)

    print("\nCreating obstacles:\n----------")
    # Adds obstacles (e.g., barriers) to the map
    obstacles = map.get_obstacles(fig)
    print("obstacles:", obstacles)

    print("\nCreating vehicles:\n----------")
    L = 4.8
    B = 1.8
    # Places static and moving vehicles on the map, init_movingvehis- Initial states of moving vehicles.
    vehicles, init_movingvehis, moving_vehicles = map.get_vehicles(fig, L, B)
    print("vehicles:", vehicles)
    print("moving vehicles:", moving_vehicles)  #Active moving vehicles.

    print("\nSetting start and target:\n----------")
    starts, targets = map.get_startandtarget()
    print("starts:", starts, ", targets:", targets)

    # Initialize vehicle states for multiple vehicles
    rear_positions = [[start[0], start[1]] for start in starts]
    front_positions = [[start[0] + 0.5, start[1] + 0.5] for start in starts]  # Slightly offset for front position
    orientations = [0, 0]  # Initial orientations for both vehicles
    iterations = [0, 0]  # Iteration counters for both vehicles

    # Vehicle model
    print("\nEstablishing the vehicle model:\n----------")
    l = 3 #Wheelbase
    velocity = 20  #vehicle speed
    cars = [CAR(L, B, l, velocity) for _ in range(len(starts))]  # Create a CAR instance for each vehicle

    # Verify consistency in vehicle positioning
    for i, car in enumerate(cars):
        # Pass the entire list of rear and front positions for all vehicles
        rear_p = rear_positions[i]
        front_p = front_positions[i]
        print("for vehicle: ",i)
        print(f"rear_p: {rear_p}, type: {type(rear_p)}")
        print(f"front_p: {front_p}, type: {type(front_p)}")


        # Pass the 2D lists to difference method
        dif_distance_list, dif_distance2_list = car.difference(rear_p, front_p, 0)
        
        # Fix: Subtract corresponding elements of the two lists
        #dif_distance_diff = [d1 - d2 for d1, d2 in zip(dif_distance_list, dif_distance2_list)]
        
        #print(f"Vehicle {i + 1}: Difference between formula and subtract:", dif_distance_diff)
        #print(f"Vehicle {i + 1}: Distance difference:", dif_distance_list)

        car.draw_car(rear_p, front_p) # Draw the current vehicle's position

    # APF path planning algorithm
    t1 = time.time()

    print("\nAPF algorithm path planning:\n----------")
    k_att = 40
    k_rep = 10
    r_apf = 5
    k_replane = 5
    k_lane = 10
    k_car = 500
    lane_width = 3.5
    target_area = 3.5
    F_e = 50
    del_t = 0.05  # Changeable
    max_iters = 105

    print("STARTING In Zong.py:", rear_positions, front_positions, orientations,moving_vehicles)
    # Initialize APF for multiple vehicles
    apf = APF(
        k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, target_area, F_e, del_t, max_iters,
        solid_lines, transition_lines, obstacles, vehicles, L, B, cars[0]  # Passing one car instance as a model
    )

    # Perform path planning for all vehicles
    rear_positions, front_positions, orientations, iterations, moving_vehicles = apf.pathplanning(
        rear_positions, front_positions, orientations, targets, iterations, moving_vehicles
    )

    t2 = time.time()
    apf_time = t2 - t1

    # Display results, final vehicle states
    for i in range(len(starts)):
        print(f"\nVehicle {i + 1} Results:")
        print(f"  Total rear velocity: {apf.rear_velocities[i]}")
        print(f"  Total path: {apf.paths[i]}")
        print(f"  Total direction: {apf.directions[i]}")
        print(f"  Total front velocity: {apf.front_velocities[i]}")
        print(f"  Total front position: {apf.front_points[i]}")
        print(f"  Total steering: {apf.steerings[i]}")
        print(f"  Total iterations: {iterations[i]}")
        
        # Print forces for debugging
        print(f"  Attractive forces for Vehicle {i + 1}: {apf.F_atts[i]}")
        print(f"  Lane forces for Vehicle {i + 1}: {apf.F_lanes[i]}")
        print(f"  Obstacle forces for Vehicle {i + 1}: {apf.F_obss[i]}")
        print(f"  Vehicle repulsion forces for Vehicle {i + 1}: {apf.F_vehs[i]}")
        print(f"  Total forces for Vehicle {i + 1}: {apf.F_totals[i]}")
    print("\nAPF execution time:", apf_time)

    # Visualize the final paths and states
    map.final_draw(fig, apf.front_points, apf.paths, init_movingvehis, moving_vehicles, L, B)
    plt.pause(0.1)

    plt.show()
