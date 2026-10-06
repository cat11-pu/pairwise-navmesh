"""Tests for the navmesh kernel (unittest, standard library only)."""

import math
import unittest

from navmesh.core import NavMesh, path_length, point_in_region, simplify


def square(x, y, size=1.0):
    """Counter clockwise corners of an axis aligned square."""
    return [
        (x, y),
        (x + size, y),
        (x + size, y + size),
        (x, y + size),
    ]


def row_mesh():
    """Three two by two regions in a row."""
    return NavMesh(
        [square(0.0, 0.0, 2.0), square(2.0, 0.0, 2.0), square(4.0, 0.0, 2.0)]
    )


def corner_mesh():
    """A two by one bar, a one by one square and a one by two tower."""
    return NavMesh(
        [
            [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)],
            [(2.0, 0.0), (3.0, 0.0), (3.0, 1.0), (2.0, 1.0)],
            [(2.0, 1.0), (3.0, 1.0), (3.0, 3.0), (2.0, 3.0)],
        ]
    )


def zigzag_mesh():
    """Six unit regions: right along the bottom, up, then left along the top."""
    return NavMesh(
        [
            square(0.0, 0.0),
            square(1.0, 0.0),
            square(2.0, 0.0),
            square(2.0, 1.0),
            square(2.0, 2.0),
            square(1.0, 2.0),
        ]
    )


def split_mesh():
    """Two joined regions and a third one far away."""
    return NavMesh([square(0.0, 0.0), square(1.0, 0.0), square(5.0, 0.0)])


class RouteTests(unittest.TestCase):
    def test_a_straight_route_is_a_single_segment(self):
        path = row_mesh().find_path((0.2, 1.0), (5.5, 1.0))
        self.assertEqual(path, [(0.2, 1.0), (5.5, 1.0)])
        self.assertAlmostEqual(path_length(path), 5.3, places=9)

    def test_a_route_around_a_corner_turns_at_the_inner_vertex(self):
        path = corner_mesh().find_path((0.5, 0.5), (2.5, 2.5))
        self.assertEqual(path, [(0.5, 0.5), (2.0, 1.0), (2.5, 2.5)])

    def test_a_route_with_two_turns_keeps_both_corners(self):
        path = zigzag_mesh().find_path((0.5, 0.5), (1.5, 2.5))
        self.assertEqual(
            path, [(0.5, 0.5), (2.0, 1.0), (2.0, 2.0), (1.5, 2.5)]
        )
        measured = 0.0
        for first, second in zip(path, path[1:]):
            measured += math.hypot(second[0] - first[0], second[1] - first[1])
        self.assertAlmostEqual(
            measured, math.sqrt(2.5) + 1.0 + math.sqrt(0.5), places=9
        )

    def test_a_route_across_regions_lists_every_region(self):
        mesh = zigzag_mesh()
        self.assertEqual(
            mesh.find_region_path((0.5, 0.5), (1.5, 2.5)), [0, 1, 2, 3, 4, 5]
        )

    def test_a_route_back_visits_the_same_regions_in_reverse(self):
        mesh = zigzag_mesh()
        self.assertEqual(
            mesh.find_region_path((1.5, 2.5), (0.5, 0.5)), [5, 4, 3, 2, 1, 0]
        )
        forward = mesh.find_path((0.5, 0.5), (1.5, 2.5))
        backward = mesh.find_path((1.5, 2.5), (0.5, 0.5))
        self.assertEqual(forward, [(0.5, 0.5), (2.0, 1.0), (2.0, 2.0), (1.5, 2.5)])
        self.assertEqual(backward, list(reversed(forward)))

    def test_a_point_on_a_shared_edge_belongs_to_a_region(self):
        mesh = row_mesh()
        self.assertIn(mesh.locate((2.0, 1.0)), (0, 1))
        self.assertEqual(
            mesh.find_path((2.0, 1.0), (5.5, 1.0)), [(2.0, 1.0), (5.5, 1.0)]
        )

    def test_an_unreachable_goal_reports_no_route(self):
        mesh = split_mesh()
        self.assertIsNone(mesh.find_path((0.5, 0.5), (5.5, 0.5)))


class HelperTests(unittest.TestCase):
    def test_route_length_is_the_sum_of_its_segments(self):
        self.assertAlmostEqual(path_length([(0.0, 0.0), (3.0, 4.0)]), 5.0, places=9)
        self.assertAlmostEqual(
            path_length([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]), 2.0, places=9
        )
        self.assertAlmostEqual(path_length([(2.0, 2.0)]), 0.0, places=9)
        self.assertIsNone(path_length(None))

    def test_simplify_drops_duplicates_and_needless_waypoints(self):
        self.assertEqual(
            simplify([(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0)]),
            [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)],
        )
        self.assertEqual(
            simplify([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]),
            [(0.0, 0.0), (3.0, 0.0)],
        )
        self.assertEqual(
            simplify([(0.0, 0.0), (2.0, 0.0), (1.0, 0.0), (1.0, 1.0)]),
            [(0.0, 0.0), (2.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        )

    def test_points_outside_the_mesh_are_rejected(self):
        mesh = row_mesh()
        self.assertTrue(point_in_region(square(0.0, 0.0), (0.5, 0.5)))
        self.assertFalse(point_in_region(square(0.0, 0.0), (3.0, 0.5)))
        self.assertEqual(mesh.locate((0.5, 1.0)), 0)
        self.assertIsNone(mesh.locate((9.0, 9.0)))
        with self.assertRaises(ValueError):
            mesh.find_path((9.0, 9.0), (0.5, 1.0))
        with self.assertRaises(ValueError):
            mesh.find_path((0.5, 1.0), (9.0, 9.0))

        zigzag = zigzag_mesh()
        start = (0.5, 0.5)
        path = zigzag.find_path(start, (1.5, 2.5))
        self.assertEqual(path[0], start)
        corners = set()
        for region in zigzag.regions:
            corners.update(region)
        for point in path:
            self.assertIn(point, corners | {start, (1.5, 2.5)})


if __name__ == "__main__":
    unittest.main()
