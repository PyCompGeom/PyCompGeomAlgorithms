from PyCompGeomAlgorithms.core import BinTree


def test_bin_tree_height():
    tree = BinTree.from_iterable([1, 2, 3, 4, 5, 6])
    assert tree.root.height == 2
    assert tree.root.left.height == 1
    assert tree.root.left.right.height == 0
    assert tree.root.right.height == 1
    assert tree.root.right.left.height == 0
    assert tree.root.right.right.height == 0
