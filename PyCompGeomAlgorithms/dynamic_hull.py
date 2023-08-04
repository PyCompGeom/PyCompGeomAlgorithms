from .core import BinTreeNode, BinTree, ThreadedBinTree, ThreadedBinTreeNode, PointType, Point
from .jarvis import jarvis


class DynamicHullNode(BinTreeNode):
    def __init__(self, data, subhull=None, left_supporting_index=0):
        super().__init__(data)
        self.subhull = subhull
        self.left_supporting_index = left_supporting_index
    
    @property
    def point(self):
        return self.data

    @property
    def is_point(self):
        return self.subhull.root.is_leaf

    @property
    def is_segment(self):
        root = self.subhull.root
        return not root.left and root.right and root.right.is_leaf
    
    def __eq__(self, other):
        return (
            super().__eq__(other)
            and self.subhull == other.subhull
            and self.left_supporting_index == other.left_supporting_index
        )
    
    def __repr__(self):
        return ", ".join(repr(node.point) for node in self.subhull.traverse_inorder())


class SubHullNode(ThreadedBinTreeNode):
    @property
    def point(self):
        return self.data


class SubHullThreadedBinTree(ThreadedBinTree):
    node_class = SubHullNode

    @classmethod
    def from_iterable(cls, iterable, circular=False):
        return super().from_iterable(iterable, circular)


def upper_dynamic_hull(points, point_to_insert_or_delete):
    points.sort()
    # yield points
    
    tree = BinTree(make_dh_tree_root(points))
    # yield tree

    optimize_dynamic_hull_tree(tree.root)
    # yield tree

    if point_to_insert_or_delete in points:
        delete_point(point_to_insert_or_delete, tree)
    else:
        insert_point(point_to_insert_or_delete, tree)

    return [n.point for n in tree.root.subhull.traverse_inorder()]


def make_dh_tree_root(points, i=None, n=None):
    if i is None:
        i = [0]
    if n is None:
        n = len(points)
    if n == 1:
        res = DynamicHullNode(points[i[0]], SubHullThreadedBinTree.from_iterable([points[i[0]]]))
        i[0] += 1
        return res

    n_right = n // 2
    n_left = n - n_right
    left, right = make_dh_tree_root(points, i, n_left), make_dh_tree_root(points, i, n_right)

    return merge(left, right)


def merge(node1, node2):
    if node1.is_point or node2.is_point or node1.is_segment or node2.is_segment:
        return merge_trivial(node1, node2)

    subhull_node1, subhull_node2 = node1.subhull.root, node2.subhull.root
    prev1, prev2 = None, None
    while prev1 != subhull_node1 or prev2 != subhull_node2:
        prev1, prev2 = subhull_node1, subhull_node2
        subhull_node1, subhull_node2 = next_nodes(subhull_node1, subhull_node2)
    
    subhull_nodes1, subhull_nodes2 = node1.subhull.traverse_inorder(), node2.subhull.traverse_inorder()
    subhull = [
        node.point for node in
        subhull_nodes1[:subhull_nodes1.index(prev1)+1] + subhull_nodes2[subhull_nodes2.index(prev2):]
    ]
    
    joint_node = DynamicHullNode(
        data=node1.rightmost_node,
        subhull=SubHullThreadedBinTree.from_iterable(subhull),
        left_supporting_index=subhull.index(prev1.point)
    )
    joint_node.left, joint_node.right = node1, node2
    
    return joint_node


def merge_trivial(node1, node2):
    """
        Merge two nodes by constructing the convex hull of their points with Jarvis' algorithm and extracting its upper part.
        
        For efficiency's sake, only use in trivial cases, such as point & point, segment & point, segment & segment, and segment & 3-chain.
    """
    subhull_root1, subhull_root2 = node1.subhull.root, node2.subhull.root
    points1 = [node.point for node in subhull_root1.traverse_inorder()]
    points2 = [node.point for node in subhull_root2.traverse_inorder()]
    points = points1 + points2
    subhull = (
        ([] if len(points) == 2 and points[0].x == points[1].x else [points[0]]) +
        [p for p in jarvis(points) if Point.direction(points[0], points[-1], p) < 0] +
        [points[-1]]
    )
    
    rightmost_point_in_left_subtree = node1.rightmost_node.point
    left_supporting_point = subhull[0] if len(subhull) == 1 else next(p for p in reversed(points1) if p in subhull)
    joint_node = DynamicHullNode(
        data=rightmost_point_in_left_subtree,
        subhull=SubHullThreadedBinTree.from_iterable(subhull),
        left_supporting_index=subhull.index(left_supporting_point)
    )
    joint_node.left, joint_node.right = node1, node2
    
    return joint_node


def next_nodes(node1, node2):
    type1, type2 = PointType.by_nodes(node2, node1), PointType.by_nodes(node1, node2)
    return next_left_node(node1, type1), next_right_node(node2, type2)


def next_left_node(node, point_type):
    return {
        PointType.reflex: node.right,
        PointType.right_supporting: node,
        PointType.convex: node.left
    }[point_type]


def next_right_node(node, point_type):
    return {
        PointType.reflex: node.left,
        PointType.left_supporting: node,
        PointType.convex: node.right
    }[point_type]


def optimize_dynamic_hull_tree(node, parent_node=None):
    if node.left:
        optimize_dynamic_hull_tree(node.left, node)
    if node.right:
        optimize_dynamic_hull_tree(node.right, node)
    if parent_node:
        optimize_subhull(node, parent_node)
    

def optimize_subhull(node, parent_node):
    node_points = [n.point for n in node.subhull.traverse_inorder()]
    parent_points = {n.point for n in parent_node.subhull.traverse_inorder()}

    filtered_points = [p for p in node_points if p not in parent_points]
    node.subhull = SubHullThreadedBinTree.from_iterable(filtered_points)


def insert_point(point, tree):
    pass


def delete_point(point, tree):
    pass
