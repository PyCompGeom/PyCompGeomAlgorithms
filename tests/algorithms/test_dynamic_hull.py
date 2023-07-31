from PyCompGeomAlgorithms.core import Point
from PyCompGeomAlgorithms.dynamic_hull import DynamicHullNode, SubHullThreadedBinTree, upper_dynamic_hull, merge


def test_dynamic_hull1(self):
    pts = [Point(3, 3), Point(1, 1), Point(5, 0)]
    hull = [Point(1, 1), Point(3, 3), Point(5, 0)]

    self.assertEqual(hull, upper_dynamic_hull(pts, None))


def test_dynamic_hull2(self):
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
    hull = [
        Point(0, 0),
        Point(2, 8),
        Point(3, 10),
        Point(6, 8),
        Point(7, 7),
        Point(10, 3)
    ]
    
    self.assertEqual(hull, upper_dynamic_hull(pts, None))


def test_dynamic_hull3(self):
    pts = [Point(0, 2), Point(0, 4)]
    hull = [Point(0, 4)]

    self.assertEqual(hull, upper_dynamic_hull(pts, None))


def test_dynamic_hull4(self):
    """Vertical line, only upper point"""
    pts = [Point(0, i) for i in range(5)]
    hull = [Point(0, 4)]

    self.assertEqual(hull, upper_dynamic_hull(pts, None))


def test_dynamic_hull5(self):
    """Horizontal line, segment of only extreme left and right points"""
    pts = [Point(i, 0) for i in range(5)]
    hull = [Point(0, 0), Point(4, 0)]

    self.assertEqual(hull, upper_dynamic_hull(pts, None))


def test_merge_segments1(self):
    """Hull is p1-p4"""
    pts = [Point(0, 2), Point(1, 0), Point(3, 1), Point(4, 3)]
    p1, p2, p3, p4 = pts

    segment_node1, segment_node2 = self._make_two_segment_nodes(pts)
    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p4]))
    joint_node.left, joint_node.right = segment_node1, segment_node2

    self.assertEqual(joint_node, merge(segment_node1, segment_node2))


def test_merge_segments2(self):
    """Hull is p1-p2-p4, p3 is below p1-p4"""
    pts = [Point(1, 1), Point(2, 4), Point(3, 1), Point(4, 2)]
    p1, p2, p3, p4 = pts

    segment_node1, segment_node2 = self._make_two_segment_nodes(pts)
    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p2, p4]), 1)
    joint_node.left, joint_node.right = segment_node1, segment_node2

    self.assertEqual(joint_node, merge(segment_node1, segment_node2))


def test_merge_segments3(self):
    """Hull is p1-p2-p4, p3 is above p1-p4"""
    pts = [Point(1, 1), Point(2, 4), Point(3, 2), Point(4, 2)]
    p1, p2, p3, p4 = pts

    segment_node1, segment_node2 = self._make_two_segment_nodes(pts)
    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p2, p4]), 1)
    joint_node.left, joint_node.right = segment_node1, segment_node2

    self.assertEqual(joint_node, merge(segment_node1, segment_node2))


def test_merge_segments4(self):
    """Hull is p1-p3-p4, p2 is below p1-p4"""
    pts = [Point(1, 3), Point(2, 1), Point(3, 3), Point(4, 1)]
    p1, p2, p3, p4 = pts

    segment_node1, segment_node2 = self._make_two_segment_nodes(pts)
    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p3, p4]))
    joint_node.left, joint_node.right = segment_node1, segment_node2

    self.assertEqual(joint_node, merge(segment_node1, segment_node2))


def test_merge_segments5(self):
    """Hull is p1-p3-p4, p2 is above p1-p4"""
    pts = [Point(1, 3), Point(2, 3), Point(3, 4), Point(4, 1)]
    p1, p2, p3, p4 = pts

    segment_node1, segment_node2 = self._make_two_segment_nodes(pts)
    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p3, p4]))
    joint_node.left, joint_node.right = segment_node1, segment_node2

    self.assertEqual(joint_node, merge(segment_node1, segment_node2))


def test_merge_segments6(self):
    """Hull is p1-p2-p3-p4"""
    pts = [Point(1, 1), Point(2, 3), Point(3, 3), Point(4, 1)]
    p1, p2, p3, p4 = pts

    segment_node1, segment_node2 = self._make_two_segment_nodes(pts)
    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p2, p3, p4]), 1)
    joint_node.left, joint_node.right = segment_node1, segment_node2

    self.assertEqual(joint_node, merge(segment_node1, segment_node2))


def test_merge_segment_and_point1(self):
    """Segment p1-p2 and point p3, where p2 is above p1-p3."""
    pts = [Point(0, 1), Point(2, 0), Point(5, 5)]
    p1, p2, p3 = pts
    n1, n2, n3 = [DynamicHullNode(p, SubHullThreadedBinTree.from_iterable([p])) for p in pts]
    segment_node = DynamicHullNode(p1, SubHullThreadedBinTree.from_iterable([p1, p2]))
    segment_node.left, segment_node.right = n1, n2
    point_node = DynamicHullNode(p3, SubHullThreadedBinTree.from_iterable([p3]))

    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p3]))
    joint_node.left, joint_node.right = segment_node, n3

    self.assertEqual(joint_node, merge(segment_node, point_node))


def test_merge_segment_and_point2(self):
    """Segment p1-p2 and point p3, where p2 is above p1-p3."""
    pts = [Point(0, 1), Point(2, 3), Point(5, 0)]
    p1, p2, p3 = pts
    n1, n2, n3 = [DynamicHullNode(p, SubHullThreadedBinTree.from_iterable([p])) for p in pts]
    segment_node = DynamicHullNode(p1, SubHullThreadedBinTree.from_iterable([p1, p2]))
    segment_node.left, segment_node.right = n1, n2
    point_node = DynamicHullNode(p3, SubHullThreadedBinTree.from_iterable([p3]))

    joint_node = DynamicHullNode(p2, SubHullThreadedBinTree.from_iterable([p1, p2, p3]), 1)
    joint_node.left, joint_node.right = segment_node, n3

    self.assertEqual(joint_node, merge(segment_node, point_node))


def _make_two_segment_nodes(self, points):
    p1, p2, p3, p4 = points
    n1, n2, n3, n4 = [DynamicHullNode(p, SubHullThreadedBinTree.from_iterable([p])) for p in points]

    segment_node1 = DynamicHullNode(p1, SubHullThreadedBinTree.from_iterable([p1, p2]))
    segment_node1.left, segment_node1.right = n1, n2
    segment_node2 = DynamicHullNode(p3, SubHullThreadedBinTree.from_iterable([p3, p4]))
    segment_node2.left, segment_node2.right = n3, n4
    
    return segment_node1, segment_node2
