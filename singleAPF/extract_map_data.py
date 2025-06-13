import pandas as pd
import json

# Load annotations
file_path = "C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/annotations.csv"
df = pd.read_csv(file_path)

# Convert class names to lowercase to avoid case mismatches
df["class"] = df["class"].str.lower()

# Extract frame numbers from filenames
df["frame"] = df["filename"].str.extract(r"MP4-(\d+)_jpg").astype(int)

# Select one frame for the map (e.g., Frame 2)
map_frame = 2
df_map = df[df["frame"] == map_frame]

# Extract boundaries and obstacles
boundaries = df_map[df_map["class"] == "boundaries"]
obstacles = df_map[df_map["class"] == "obstacle"]

# Function to get all polygon points for each object
def extract_polygon_data(df):
    polygons = []
    for _, row in df.iterrows():
        polygons.append({"points": [[row["xmin"], row["ymin"]], [row["xmax"], row["ymin"]],
                                    [row["xmax"], row["ymax"]], [row["xmin"], row["ymax"]]]})
    return polygons

# Store extracted map data
map_data = {
    "boundaries": extract_polygon_data(boundaries),
    "obstacles": extract_polygon_data(obstacles)
}

# Save extracted map data as JSON
output_file = "map_data.json"
with open(output_file, "w") as f:
    json.dump(map_data, f, indent=4)

print(f"✅ Map data saved in {output_file} using frame {map_frame}")
