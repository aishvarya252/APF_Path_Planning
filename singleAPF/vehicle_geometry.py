import math

def compute_corners(rear_x, rear_y, theta, L, B):
    # Compute front center
    front_x = rear_x + L * math.cos(theta)
    front_y = rear_y + L * math.sin(theta)

    # Half-width offsets
    dx = (B / 2) * math.sin(theta)
    dy = (B / 2) * math.cos(theta)

    # Rear corners
    rear_left  = (rear_x + dx, rear_y - dy)
    rear_right = (rear_x - dx, rear_y + dy)

    # Front corners
    front_left  = (front_x + dx, front_y - dy)
    front_right = (front_x - dx, front_y + dy)

    return [rear_left, rear_right, front_right, front_left]  # Clockwise or counterclockwise
