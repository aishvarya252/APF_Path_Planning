def midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

def calculate_axle_centers(vehicle_id, front_points, rear_points):
    """
    front_points: tuple of two (x, y) coordinates representing front corners
    rear_points: tuple of two (x, y) coordinates representing rear corners
    """
    if len(front_points) != 2 or len(rear_points) != 2:
        raise ValueError("Exactly two points required for both front and rear.")

    front_center = midpoint(front_points[0], front_points[1])
    rear_center = midpoint(rear_points[0], rear_points[1])

    return {
        "vehicle_id": vehicle_id,
        "front_axle_center": front_center,
        "rear_axle_center": rear_center
    }

# Example usage:
vehicle_id = "4"
front = [(1135.015,609.798), (1129.83,620.174)]
rear = [(1104.644,606.833), (1111.311,595.716)]

result = calculate_axle_centers(vehicle_id, front, rear)
print(result)
