import pandas as pd

# Load the vehicle positions CSV file
file_path = "C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/vehicle_positions.csv"
df = pd.read_csv(file_path)

# Calculate front and rear axle center coordinates
def compute_axle_centers(df):
    results = []
    
    for _, row in df.iterrows():
        vehicle_id = row["vehicle_id"]
        
        # Rear axle center (midpoint of rear-left and rear-right corners)
        rear_x_start = (row["rear_left_x_start"] + row["rear_right_x_start"]) / 2
        rear_y_start = (row["rear_left_y_start"] + row["rear_right_y_start"]) / 2
        rear_x_end = (row["rear_left_x_end"] + row["rear_right_x_end"]) / 2
        rear_y_end = (row["rear_left_y_end"] + row["rear_right_y_end"]) / 2
        
        # Front axle center (midpoint of front-left and front-right corners)
        front_x_start = (row["front_left_x_start"] + row["front_right_x_start"]) / 2
        front_y_start = (row["front_left_y_start"] + row["front_right_y_start"]) / 2
        front_x_end = (row["front_left_x_end"] + row["front_right_x_end"]) / 2
        front_y_end = (row["front_left_y_end"] + row["front_right_y_end"]) / 2
        
        # Compute length and breadth of the vehicle
        length = ((front_x_start - rear_x_start) ** 2 + (front_y_start - rear_y_start) ** 2) ** 0.5
        breadth = ((row["rear_left_x_start"] - row["rear_right_x_start"]) ** 2 +
                   (row["rear_left_y_start"] - row["rear_right_y_start"]) ** 2) ** 0.5
        
        results.append([vehicle_id, rear_x_start, rear_y_start, front_x_start, front_y_start,
                        rear_x_end, rear_y_end, front_x_end, front_y_end, length, breadth])
    
    return pd.DataFrame(results, columns=["vehicle_id", "rear_x_start", "rear_y_start", "front_x_start", "front_y_start",
                                          "rear_x_end", "rear_y_end", "front_x_end", "front_y_end", "length", "breadth"])

# Compute the axle centers
df_axles = compute_axle_centers(df)

# Ensure uniform length and breadth for each vehicle
average_length = df_axles["length"].mean()
average_breadth = df_axles["breadth"].mean()
df_axles["length"] = average_length
df_axles["breadth"] = average_breadth

# Save the processed data
df_axles.to_csv("C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/vehicle_axle_pos.csv", index=False)
print("Axle positions computed and saved.")
