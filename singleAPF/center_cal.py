'''import pandas as pd

def midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

def compute_axle_centers(csv_path):
    df = pd.read_csv(csv_path)

    results = []

    for frame_name in df['frame_name'].unique():
        frame_data = df[df['frame_name'] == frame_name]

        for category_id in frame_data['category_id'].unique():
            vehicle_data = frame_data[frame_data['category_id'] == category_id]

            if len(vehicle_data) < 4:
                continue  # skip incomplete vehicles

            coords = vehicle_data[['x', 'y']].values

            # First two = front, next two = rear
            front = [tuple(coords[0]), tuple(coords[1])]
            rear = [tuple(coords[2]), tuple(coords[3])]

            front_center = midpoint(front[0], front[1])
            rear_center = midpoint(rear[0], rear[1])

            results.append({
                'frame': frame_name,
                'vehicle_id': category_id,
                'front_axle_center_x': front_center[0],
                'front_axle_center_y': front_center[1],
                'rear_axle_center_x': rear_center[0],
                'rear_axle_center_y': rear_center[1],
            })

    return pd.DataFrame(results)

# === Example usage ===

csv_file_path = 'C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/vehicle_segmentation_coordinates.csv'  # Replace with your actual file path
axle_centers_df = compute_axle_centers(csv_file_path)

# Show or save the result
print(axle_centers_df)

# Optional: save to CSV
axle_centers_df.to_csv('axle_centers_output.csv', index=False)'''

'''
#code for computing axle centers from vehicle segmentation coordinates and saving to CSV
import pandas as pd
import math

def midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

def compute_axle_centers(csv_path):
    df = pd.read_csv(csv_path)

    data_dict = {}

    for frame_name in ['DJI_0010_MP4-0001', 'DJI_0010_MP4-0030']:  # Only first and last
        frame_data = df[df['frame_name'] == frame_name]

        for category_id in frame_data['category_id'].unique():
            vehicle_data = frame_data[frame_data['category_id'] == category_id]

            if len(vehicle_data) < 4:
                continue

            coords = vehicle_data[['x', 'y']].values

            front = [tuple(coords[0]), tuple(coords[1])]
            rear = [tuple(coords[2]), tuple(coords[3])]

            front_center = midpoint(front[0], front[1])
            rear_center = midpoint(rear[0], rear[1])

            if category_id not in data_dict:
                data_dict[category_id] = {}

            data_dict[category_id][frame_name] = {
                'front': front_center,
                'rear': rear_center
            }

    # Create final output
    output = []
    for vehicle_id, frames in data_dict.items():
        if 'DJI_0010_MP4-0001' in frames and 'DJI_0010_MP4-0030' in frames:
            output.append({
                'vehicle_id': vehicle_id,
                'start_front_x': frames['DJI_0010_MP4-0001']['front'][0],
                'start_front_y': frames['DJI_0010_MP4-0001']['front'][1],
                'start_rear_x': frames['DJI_0010_MP4-0001']['rear'][0],
                'start_rear_y': frames['DJI_0010_MP4-0001']['rear'][1],
                'end_front_x': frames['DJI_0010_MP4-0030']['front'][0],
                'end_front_y': frames['DJI_0010_MP4-0030']['front'][1],
                'end_rear_x': frames['DJI_0010_MP4-0030']['rear'][0],
                'end_rear_y': frames['DJI_0010_MP4-0030']['rear'][1],
            })

    return pd.DataFrame(output)

# === Example usage ===

csv_file_path = 'C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/vehicle_segmentation_coordinates.csv'  # Replace with your actual file path
summary_df = compute_axle_centers(csv_file_path)

# Show or save the result
print(summary_df)

# Optional: save to CSV
summary_df.to_csv('vehicle_axle_movement_summary.csv', index=False)'''

#code for computing vehicle dimensions from vehicle segmentation coordinates and saving to CSV
# This code computes the vehicle dimensions (length and breadth) from the vehicle segmentation coordinates using avg for both start and end frames.
import pandas as pd
import math

def midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def compute_axle_centers(csv_path):
    df = pd.read_csv(csv_path)

    data_dict = {}

    for frame_name in ['DJI_0010_MP4-0001', 'DJI_0010_MP4-0030']:  # Only first and last
        frame_data = df[df['frame_name'] == frame_name]

        for category_id in frame_data['category_id'].unique():
            vehicle_data = frame_data[frame_data['category_id'] == category_id]

            if len(vehicle_data) < 4:
                continue

            coords = vehicle_data[['x', 'y']].values

            # Front and rear points
            front = [tuple(coords[0]), tuple(coords[1])]
            rear = [tuple(coords[2]), tuple(coords[3])]

            # Centers
            front_center = midpoint(front[0], front[1])
            rear_center = midpoint(rear[0], rear[1])

            # Distances
            vehicle_length = distance(front_center, rear_center)
            front_width = distance(front[0], front[1])
            rear_width = distance(rear[0], rear[1])
            vehicle_breadth = (front_width + rear_width) / 2

            if category_id not in data_dict:
                data_dict[category_id] = {}

            data_dict[category_id][frame_name] = {
                'front': front_center,
                'rear': rear_center,
                'length': vehicle_length,
                'breadth': vehicle_breadth
            }

    # Create final output
    output = []
    for vehicle_id, frames in data_dict.items():
        if 'DJI_0010_MP4-0001' in frames and 'DJI_0010_MP4-0030' in frames:
            avg_length = (frames['DJI_0010_MP4-0001']['length'] + frames['DJI_0010_MP4-0030']['length']) / 2
            avg_breadth = (frames['DJI_0010_MP4-0001']['breadth'] + frames['DJI_0010_MP4-0030']['breadth']) / 2
            
            output.append({
                'vehicle_id': vehicle_id,

                'start_front_x': frames['DJI_0010_MP4-0001']['front'][0],
                'start_front_y': frames['DJI_0010_MP4-0001']['front'][1],
                'start_rear_x': frames['DJI_0010_MP4-0001']['rear'][0],
                'start_rear_y': frames['DJI_0010_MP4-0001']['rear'][1],

                'end_front_x': frames['DJI_0010_MP4-0030']['front'][0],
                'end_front_y': frames['DJI_0010_MP4-0030']['front'][1],
                'end_rear_x': frames['DJI_0010_MP4-0030']['rear'][0],
                'end_rear_y': frames['DJI_0010_MP4-0030']['rear'][1],
                
                'vehicle_length': avg_length,
                'vehicle_breadth': avg_breadth
            })

    return pd.DataFrame(output)

# === Example usage ===

csv_file_path = 'C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/Cases/Case2/vehicle_segmentation_coordinates.csv'  # Replace with your actual file path
summary_df = compute_axle_centers(csv_file_path)

# Show or save the result
print(summary_df)

# Optional: save to CSV
summary_df.to_csv('vehicle_dims.csv', index=False)


