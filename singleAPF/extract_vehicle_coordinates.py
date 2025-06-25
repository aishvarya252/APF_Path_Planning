import json
import csv

# Load JSON
with open('C:/ML/IHUBdata/apf/test5/A-hybrid-path-planning-algorithm-based-on-APF-and-Astar/singleAPF/Utility/Cases/Case2/train/_annotations.coco.json', 'r') as f:
    data = json.load(f)

# Target frame base names
start_frame = "DJI_0010_MP4-0001"
end_frame = "DJI_0010_MP4-0030"

# Category name filter
target_categories = [f"vehicles{i}" for i in range(1, 11)]

# Map category_id -> name
category_id_to_name = {cat["id"]: cat["name"] for cat in data["categories"]}
target_category_ids = [cid for cid, name in category_id_to_name.items() if name in target_categories]

# Find image IDs for target frames
target_image_ids = {}
for img in data["images"]:
    base_name = img["file_name"].split("_jpg")[0]
    if base_name == start_frame or base_name == end_frame:
        target_image_ids[img["id"]] = base_name

# Extract segmentation data
rows = []
for ann in data["annotations"]:
    if ann["image_id"] in target_image_ids and ann["category_id"] in target_category_ids:
        segments = ann.get("segmentation", [])
        for seg in segments:
            # Flattened list of [x1, y1, x2, y2, ..., xn, yn]
            for i in range(0, len(seg), 2):
                x, y = seg[i], seg[i+1]
                rows.append({
                    "frame_name": target_image_ids[ann["image_id"]],
                    "category_id": ann["category_id"],
                    "x": x,
                    "y": y
                })

# Save to CSV
with open('vehicle_segmentation_coordinates.csv', 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=["frame_name", "category_id", "x", "y"])
    writer.writeheader()
    writer.writerows(rows)

print("✅ CSV saved as 'vehicle_segmentation_coordinates.csv'")
