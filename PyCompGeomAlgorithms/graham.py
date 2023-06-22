from math import pi
from .core import Point


def graham(points):
    if len(points) < 3:
        yield sorted(points, key=lambda p: (p.y, -p.x))
    else:
        i = 2
        while Point.direction(points[0], points[1], points[i]) == 0:
            i += 1
        
        centroid = Point.centroid(points[0], points[1], points[i])
        yield centroid

        origin = min(points, key=lambda p: (p.y, -p.x))
        ordered_points = sort_points(points, centroid, origin)
        yield ordered_points
        yield origin

        ordered_points.append(origin)
        steps_table = []
        hull = make_hull(steps_table, ordered_points)
        ordered_points.pop()
        yield steps_table
        yield hull


def sort_points(points, centroid, origin):
    min_angle = Point.polar_angle(origin, centroid)

    def angle_and_dist(p):
        p_angle = Point.polar_angle(p, centroid)
        angle = p_angle if p_angle >= min_angle else 2 * pi + p_angle
        return (angle, Point.dist(p, centroid))

    return sorted(points, key=angle_and_dist)


def make_hull(steps_table, ordered_points):
    ans = ordered_points[:2]
    for p in ordered_points[2:]:
        while len(ans) > 1 and Point.direction(ans[-2], ans[-1], p) >= 0:
            steps_table.append(current_step(ans, False, p))
            ans.pop()

        if len(ans) > 1:
            steps_table.append(current_step(ans, True, p))
        ans.append(p)

    return ans[:-1]


def current_step(ans, add, p):
    """Current step: current points' triple, add/delete, point to add/delete."""
    return [ans[-2], ans[-1], p], add


print(next(graham([Point(1,1), Point(2,2)])))