from math import isclose
from PyCompGeomAlgorithms.core import Point
from PyCompGeomAlgorithms.graham import graham


def test_graham1():
    pts = [Point(7, 0), Point(3, 3), Point(0, 0)]
    centroid = Point(3.3333333333333335, 1.0)
    ordered = [Point(7, 0), Point(3, 3), Point(0, 0)]
    origin = Point(7, 0)
    steps = [
        ([ordered[0], ordered[1], ordered[2]], True),
        ([ordered[1], ordered[2], ordered[0]], True)
    ]
    hull = [Point(7, 0), Point(3, 3), Point(0, 0)]
    
    ans = graham(pts)
    assert next(ans) == centroid
    assert next(ans) == ordered
    assert next(ans) == origin
    assert next(ans) == steps
    assert next(ans) == hull

def test_graham2():
    pts = [
        Point(3, 10),
        Point(6, 8),
        Point(3, 5),
        Point(2, 8),
        Point(4, 8),
        Point(5, 5),
        Point(3, 3),
        Point(7, 7),
        Point(5, 0),
        Point(0, 0),
        Point(10, 3),
    ]
    centroid = Point(4.0, 7.666666666666667)
    ordered = [
        Point(5, 0),
        Point(5, 5),
        Point(10, 3),
        Point(7, 7),
        Point(6, 8),
        Point(4, 8),
        Point(3, 10),
        Point(2, 8),
        Point(0, 0),
        Point(3, 5),
        Point(3, 3),
    ]
    origin = Point(5, 0)
    steps = [
        ([ordered[0], ordered[1], ordered[2]], False),
        ([ordered[0], ordered[2], ordered[3]], True),
        ([ordered[2], ordered[3], ordered[4]], True),
        ([ordered[3], ordered[4], ordered[5]], True),
        ([ordered[4], ordered[5], ordered[6]], False),
        ([ordered[3], ordered[4], ordered[6]], True),
        ([ordered[4], ordered[6], ordered[7]], True),
        ([ordered[6], ordered[7], ordered[8]], True),
        ([ordered[7], ordered[8], ordered[9]], True),
        ([ordered[8], ordered[9], ordered[10]], False),
        ([ordered[7], ordered[8], ordered[10]], True),
        ([ordered[8], ordered[10], ordered[0]], False),
        ([ordered[7], ordered[8], ordered[0]], True)
    ]
    hull = [
        Point(5, 0),
        Point(10, 3),
        Point(7, 7),
        Point(6, 8),
        Point(3, 10),
        Point(2, 8),
        Point(0, 0)
    ]
    
    ans = graham(pts)
    assert next(ans) == centroid
    assert next(ans) == ordered
    assert next(ans) == origin
    assert next(ans) == steps
    assert next(ans) == hull

def test_graham3():
    pts = [
        Point(2, 8),
        Point(5, 6),
        Point(7, 8),
        Point(8, 11),
        Point(7, 5),
        Point(10, 7),
        Point(11, 5),
        Point(8, 2),
        Point(1, 3),
        Point(5, 2),
    ]
    centroid = Point(4.666666666666667, 7.333333333333333)
    ordered = [
        Point(8, 2),
        Point(7, 5),
        Point(11, 5),
        Point(10, 7),
        Point(7, 8),
        Point(8, 11),
        Point(2, 8),
        Point(1, 3),
        Point(5, 2),
        Point(5, 6)
    ]
    origin = Point(8, 2)
    steps = [
        ([ordered[0], ordered[1], ordered[2]], False),
        ([ordered[0], ordered[2], ordered[3]], True),
        ([ordered[2], ordered[3], ordered[4]], True),
        ([ordered[3], ordered[4], ordered[5]], False),
        ([ordered[2], ordered[3], ordered[5]], False),
        ([ordered[0], ordered[2], ordered[5]], True),
        ([ordered[2], ordered[5], ordered[6]], True),
        ([ordered[5], ordered[6], ordered[7]], True),
        ([ordered[6], ordered[7], ordered[8]], True),
        ([ordered[7], ordered[8], ordered[9]], True),
        ([ordered[8], ordered[9], ordered[0]], False),
        ([ordered[7], ordered[8], ordered[0]], True)
    ]
    hull = [
        Point(8, 2),
        Point(11, 5),
        Point(8, 11),
        Point(2, 8),
        Point(1, 3),
        Point(5, 2)
    ]
    
    ans = graham(pts)
    assert next(ans) == centroid
    assert next(ans) == ordered
    assert next(ans) == origin
    assert next(ans) == steps
    assert next(ans) == hull
