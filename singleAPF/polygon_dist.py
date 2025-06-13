import math
from itertools import product

def euclidean_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def closest_point_on_segment(px, py, ax, ay, bx, by):
    """Returns the closest point from (px, py) to segment (ax, ay)-(bx, by)."""
    length_squared = (bx - ax) ** 2 + (by - ay) ** 2
    if length_squared == 0:
        return (ax, ay)
    t = max(0, min(1, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / length_squared))
    proj_x = ax + t * (bx - ax)
    proj_y = ay + t * (by - ay)
    return (proj_x, proj_y)

def segment_to_segment_closest_points(p1, p2, q1, q2):
    """Return the minimum distance and the pair of points on the two segments causing it."""
    candidates = [
        (p1, closest_point_on_segment(*p1, *q1, *q2)),
        (p2, closest_point_on_segment(*p2, *q1, *q2)),
        (q1, closest_point_on_segment(*q1, *p1, *p2)),
        (q2, closest_point_on_segment(*q2, *p1, *p2))
    ]

    min_pair = min(candidates, key=lambda pair: euclidean_distance(pair[0], pair[1]))
    min_dist = euclidean_distance(min_pair[0], min_pair[1])
    return min_dist, min_pair[0], min_pair[1]

def min_distance_between_polygons(polygon1, polygon2):
    """Return the minimum distance and the pair of closest points between two polygons."""
    edges1 = list(zip(polygon1, polygon1[1:] + polygon1[:1]))
    edges2 = list(zip(polygon2, polygon2[1:] + polygon2[:1]))

    min_dist = float('inf')
    closest_pair = (None, None)

    for (p1, p2), (q1, q2) in product(edges1, edges2):
        dist, pt1, pt2 = segment_to_segment_closest_points(p1, p2, q1, q2)
        if dist < min_dist:
            min_dist = dist
            closest_pair = (pt1, pt2)

    return min_dist, closest_pair[0], closest_pair[1]
