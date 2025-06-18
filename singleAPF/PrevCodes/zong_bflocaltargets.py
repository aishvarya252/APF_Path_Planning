import math
import matplotlib.pyplot as plt
import time
import pandas as pd
from assistant import *
from car import *
from apf import *
from plotting import create_map  # Import map generation function
import numpy as np
import json

if __name__ == "__main__":
    print("Creating a map using plotting.py:\n----------")
    fig, ax, boundaries_raw, obstacles_raw = create_map()  # Get actual map with polygons

    print("\nLoading vehicle data from CSV:\n----------")
    df = pd.read_csv("C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/vehicle_axle_with_orientation.csv")

    if len(df) < 10:
        raise ValueError("At least 10 vehicles required.")
    
    length = 3000  # or use bounding box from image
    width = 2000
    del_block = 1
    my_map = MAP(length, width, del_block)

    rear_positions = df[['start_rear_x', 'start_rear_y']].values.tolist()
    front_positions = df[['start_front_x', 'start_front_y']].values.tolist()
    targets = df[['end_rear_x', 'end_rear_y']].values.tolist()
    lengths = df['vehicle_length'].tolist()
    breadths = df['vehicle_breadth'].tolist()
    orientations = df['initial_orientation_deg'].tolist()
    orientations = [math.radians(angle) for angle in orientations]  # convert to radians
    iterations = [0] * 10


    # Convert polygon boundaries/obstacles into APF format: [x_center, y_center, width, height, orientation]
    #converting irregular polygons (from segmentation masks) into rectangles with this function:
    '''def polygon_to_apf_format(polygon):
        x_coords, y_coords = zip(*polygon)
        x_center = sum(x_coords) / len(x_coords)
        y_center = sum(y_coords) / len(y_coords)
        width = max(x_coords) - min(x_coords)
        height = max(y_coords) - min(y_coords)
        orientation = 'h' if width >= height else 'v'
        return [x_center, y_center, width, height, orientation]'''
        
    def polygon_to_apf_format_pca(polygon):
        coords = np.array(polygon)
        center = coords.mean(axis=0)
        centered_coords = coords - center

        # PCA via eigen decomposition
        cov = np.cov(centered_coords, rowvar=False)
        eigvals, eigvecs = np.linalg.eigh(cov)

        # Principal direction → eigenvector with largest eigenvalue
        principal_axis = eigvecs[:, np.argmax(eigvals)]
        angle = math.atan2(principal_axis[1], principal_axis[0])  # radians

        # Orientation based on PCA direction
        orientation = 'h' if abs(np.cos(angle)) > abs(np.sin(angle)) else 'v'

        # Compute width and height from bounds (still useful for APF)
        width = np.ptp(coords[:, 0])  # Peak-to-peak (max - min)
        height = np.ptp(coords[:, 1])

        return [center[0], center[1], width, height, orientation]

    solid_lines = [polygon_to_apf_format_pca(p) for p in boundaries_raw + obstacles_raw]
    #obstacles = [polygon_to_apf_format(p) for p in obstacles_raw]
    transition_lines = []  # No transition lines used
    vehicles = []  # No static vehicles defined for now
    moving_vehicles = []  # Can be updated if needed
    
    print("\n🚗 Vehicle Initialization:\n--------------------------")
    for i in range(10):
        print(f"Vehicle {i+1}:")
        print(f"  Rear Start: {rear_positions[i]}")
        print(f"  Front Start: {front_positions[i]}")
        print(f"  Rear Target: {targets[i]}")
        print(f"  Length = {lengths[i]}, Breadth = {breadths[i]}, Orientation = {orientations[i]}°")

    # 🔹 Plot start and target points for all vehicles using vehicle_id
    for i in range(len(rear_positions)):
        start = rear_positions[i]
        target = targets[i]
        vehicle_id = int(df.iloc[i]["vehicle_id"])  # ✅ use df here instead of vehicle_data

        # Mark Start Point
        plt.scatter(start[0], start[1], color='green', marker='o', label='Start' if i == 0 else "")
        plt.text(start[0] + 10, start[1], f"S{vehicle_id}", color='green', fontsize=8)

        # Mark Target Point
        plt.scatter(target[0], target[1], color='red', marker='x', label='Target' if i == 0 else "")
        plt.text(target[0] + 10, target[1], f"T{vehicle_id}", color='red', fontsize=8)


    # Only add legend once
    plt.legend()


    l = 3
    velocity = 40
    cars = [CAR(L, B, l, velocity) for L, B in zip(lengths, breadths)]
    
    for i in range(len(cars)):
        car = cars[i]
        rear_p = rear_positions[i]
        front_p = front_positions[i]
        car.draw_car(rear_p, front_p)

    print("\n🛣️ Starting APF Algorithm:\n--------------------------")
    k_att = 40
    k_rep = 10
    r_apf = 5
    k_replane = 5
    k_lane = 10
    k_car = 500
    lane_width = 3.5
    target_area = 3.5
    F_e = 50
    del_t = 0.05
    max_iters = 105

    t1 = time.time()
    
    print("\n🎯 Target Coordinates for All Vehicles:\n--------------------------------------")
    for i, target in enumerate(targets):
        vehicle_id = df.loc[i, "vehicle_id"]
        print(f"Vehicle ID {vehicle_id} → Target Rear Position: {target}")

    vehicle_ids = df['vehicle_id'].tolist()

    apf = APF(
        k_att, k_rep, r_apf, k_replane, k_lane, k_car, lane_width, target_area, F_e, del_t, max_iters,
        solid_lines,  # Both treated as solid
        transition_lines=[],
        obstacles=[],
        vehicles=[],
        L=lengths[0],
        B=breadths[0],
        car=cars[0],
        num_vehicles = len(rear_positions),
        vehicle_ids=vehicle_ids ,
        boundaries_raw=boundaries_raw,
        obstacles_raw=obstacles_raw)

    while not all(iterations[i] > apf.max_iters for i in range(len(rear_positions))):
        rear_positions, front_positions, orientations, iterations, moving_vehicles = apf.pathplanning(
            rear_positions, front_positions, orientations, targets, iterations, []
        )

    t2 = time.time()
    apf_time = t2 - t1

    for i in range(10):
        print(f"\n🚘 Vehicle {i + 1} Results:")
        print(f"  🔹 Rear Velocity: {apf.rear_velocities[i]}")
        print(f"  🔹 Total Path: {apf.paths[i]}")
        print(f"  🔹 Direction: {apf.directions[i]}")
        print(f"  🔹 Front Velocity: {apf.front_velocities[i]}")
        print(f"  🔹 Front Position: {apf.front_points[i]}")
        print(f"  🔹 Steering: {apf.steerings[i]}")
        print(f"  🔹 Iterations: {iterations[i]}")
        print(f"  🔸 Forces → Attractive: {apf.F_atts[i]}, Lane: {apf.F_lanes[i]}, Obstacle: {apf.F_obss[i]}, Vehicle: {apf.F_vehs[i]}, Total: {apf.F_totals[i]}")

    print("\n⏳ APF Execution Time:", apf_time)

    print("\n📍 Final Visualization:\n--------------------------")
    
    my_map.final_draw(fig, apf.front_points, apf.paths, [], [], lengths[0], breadths[0])
    plt.pause(0.1)
    plt.show()
