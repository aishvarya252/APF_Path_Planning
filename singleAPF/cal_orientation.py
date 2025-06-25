import pandas as pd
import math

def compute_orientation(row):
    dx = row['start_front_x'] - row['start_rear_x']
    dy = row['start_front_y'] - row['start_rear_y']
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

# Load your existing data (assuming you've saved it from the previous step)
df = pd.read_csv('C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/Cases/Case2/vehicle_dims.csv')

# Add orientation column
df['initial_orientation_deg'] = df.apply(compute_orientation, axis=1)

# View or save updated dataframe
print(df[['vehicle_id', 'initial_orientation_deg']])
df.to_csv('C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/Cases/Case2/vehicle_axle_with_orientation.csv', index=False)
