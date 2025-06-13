import pandas as pd

# Load the annotations CSV file
file_path = "C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/ihub.v3i.tensorflow/train/_annotations.csv"
df = pd.read_csv(file_path)

# Filter for only vehicle annotations in the correct frames
start_frame = "DJI_0010_MP4-0001_jpg"
end_frame = "DJI_0010_MP4-0030_jpg"

df_start = df[df["filename"].str.contains(start_frame) & (df["class"] == "vehicles")]
df_end = df[df["filename"].str.contains(end_frame) & (df["class"] == "vehicles")]

# Ensure both frames have exactly 10 vehicles and are sorted in order
df_start = df_start.sort_values(by=["xmin", "ymin"]).reset_index(drop=True)
df_end = df_end.sort_values(by=["xmin", "ymin"]).reset_index(drop=True)

# Check if both frames contain exactly 10 vehicles
if len(df_start) != 10 or len(df_end) != 10:
    raise ValueError(f"Incorrect number of vehicles in start ({len(df_start)}) or end ({len(df_end)}) frames.")

# Assign vehicle IDs in order
df_start["vehicle_id"] = range(1, 11)
df_end["vehicle_id"] = range(1, 11)

# Merge start and end positions based on vehicle_id
df_vehicles = pd.merge(df_start, df_end, on="vehicle_id", suffixes=("_start", "_end"))

# Extract the four bounding box corner coordinates
vehicle_positions = df_vehicles[["vehicle_id",
    "xmin_start", "ymax_start", "xmax_start", "ymax_start", "xmin_start", "ymin_start", "xmax_start", "ymin_start",
    "xmin_end", "ymax_end", "xmax_end", "ymax_end", "xmin_end", "ymin_end", "xmax_end", "ymin_end"
]]

# Rename columns for clarity
vehicle_positions.columns = [
    "vehicle_id",
    "rear_left_x_start", "rear_left_y_start", "rear_right_x_start", "rear_right_y_start",
    "front_left_x_start", "front_left_y_start", "front_right_x_start", "front_right_y_start",
    "rear_left_x_end", "rear_left_y_end", "rear_right_x_end", "rear_right_y_end",
    "front_left_x_end", "front_left_y_end", "front_right_x_end", "front_right_y_end"
]

# Display the extracted vehicle positions
vehicle_positions
print("Vehicle coordinate extraction complete.")