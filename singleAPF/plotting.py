import json
import matplotlib.pyplot as plt
import numpy as np
'''
# Load COCO JSON data (Replace 'map_data.json' with the actual JSON file)
with open('C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/_annotations.coco.json', 'r') as f:
    coco_data = json.load(f)

# Extract annotations
annotations = coco_data['annotations']

# Create a white canvas
fig, ax = plt.subplots(figsize=(15, 15))  # Increased size for better visualization
ax.set_xlim(0, 3000)  # Adjust limits based on your image size
ax.set_ylim(0, 2000)
ax.set_facecolor('white')
ax.invert_yaxis()  # Invert Y-axis to match image coordinate system

# Define colors for different categories
category_colors = {1: 'red', 2: 'blue'}

# Iterate through annotations and plot polygons for IDs 1 to 43 with image_id 0
for annotation in annotations:
    if 0 <= annotation['id'] <= 43 and annotation['image_id'] == 0:
        category_id = annotation['category_id']
        if category_id in category_colors:
            for segmentation in annotation['segmentation']:
                if len(segmentation) >= 6:  # Ensure at least 3 points (x, y pairs) for a valid polygon
                    x_coords = segmentation[0::2]  # X-coordinates
                    y_coords = segmentation[1::2]  # Y-coordinates
                    
                    # Plot polygon
                    ax.fill(x_coords, y_coords, color=category_colors[category_id], alpha=0.5, edgecolor='black')

plt.title('COCO Annotations Polygons for IDs 1 to 43 (image_id 0)')
plt.show()'''

def create_map():
    """
    Function to generate a map using COCO annotations and return the figure and axis.
    """
    # Load COCO JSON data
    with open('C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/_annotations.coco.json', 'r') as f:
        coco_data = json.load(f)

    # Extract annotations
    annotations = coco_data['annotations']

    # Create a white canvas
    fig, ax = plt.subplots(figsize=(15, 15))  # Increased size for better visualization
    ax.set_xlim(0, 3000)  # Adjust limits based on your image size
    ax.set_ylim(0, 2000)
    ax.set_facecolor('white')
    ax.invert_yaxis()  # Invert Y-axis to match image coordinate system

    # Define colors for different categories
    category_colors = {1: 'red', 2: 'blue'}
    
    boundaries = []
    obstacles = []

    # Iterate through annotations and plot polygons for IDs 1 to 43 with image_id 0
    for annotation in annotations:
        if 0 <= annotation['id'] <= 43 and annotation['image_id'] == 0:
            category_id = annotation['category_id']
            if category_id in category_colors:
                for segmentation in annotation['segmentation']:
                    if len(segmentation) >= 6:  # Ensure at least 3 points (x, y pairs) for a valid polygon
                        x_coords = segmentation[0::2]  # X-coordinates
                        y_coords = segmentation[1::2]  # Y-coordinates
                        
                        # Store as obstacle or boundary
                        if category_id == 1:
                            obstacles.append(list(zip(x_coords, y_coords)))
                        elif category_id == 2:
                            boundaries.append(list(zip(x_coords, y_coords)))

                        # Plot polygon
                        ax.fill(x_coords, y_coords, color=category_colors[category_id], alpha=0.5, edgecolor='black')
                        
    plt.title('COCO Annotations Polygons for IDs 1 to 43 (image_id 0)')
    print("\n✅ Map Debug Info:")
    '''print("Boundaries:")
    for i, b in enumerate(boundaries):
        print(f"  {i + 1}: {b}")

    print("\nObstacles:")
    for i, o in enumerate(obstacles):
        print(f"  {i + 1}: {o}")'''

    return fig, ax, boundaries, obstacles  # Return the figure and axis to be used in other scripts
#plt.show()


'''if __name__ == "__main__":
    fig, ax, boundaries, obstacles = create_map()
    plt.show()'''
