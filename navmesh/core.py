"""Navigation mesh pathfinding: convex regions, shared edges and funnel smoothing.

A mesh is a list of convex regions given as counter clockwise vertex loops.
Two regions are joined when they share a whole edge; the region graph is
searched for a sequence of neighbouring regions and the funnel algorithm
turns that sequence into a short waypoint chain.
"""

import math

EPSILON = 1e-9


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _area(a, b, c):
    """Twice the signed area of the triangle a, b, c."""
    return (c[0] - a[0]) * (b[1] - a[1]) - (b[0] - a[0]) * (c[1] - a[1])


def _side(a, b, point):
    """Signed side of a point relative to the directed line a -> b."""
    return _cross(b[0] - a[0], b[1] - a[1], point[0] - a[0], point[1] - a[1])


def _distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def polygon_edges(polygon):
    """Directed edges of a closed polygon."""
    count = len(polygon)
    return [(polygon[i], polygon[(i + 1) % count]) for i in range(count)]


def point_in_region(region, point):
    """True when the point belongs to the convex region."""
    x = float(point[0])
    y = float(point[1])
    for first, second in polygon_edges(region):
        if _side(first, second, (x, y)) <= 0.0:
            return False
    return True


def _is_collinear(a, b, c):
    """True when the three points line up."""
    return abs(_area(a, b, c)) <= EPSILON


class NavMesh(object):
    """A walkable area split into convex regions that share whole edges."""

    def __init__(self, regions):
        self.regions = [
            tuple((float(point[0]), float(point[1])) for point in region)
            for region in regions
        ]
        for region in self.regions:
            if len(region) < 3:
                raise ValueError("a region needs at least three vertices")
        self._links = {}
        self._build_links()

    def __repr__(self):
        return "NavMesh(%d regions)" % (len(self.regions),)

    def _build_links(self):
        for i in range(len(self.regions)):
            for j in range(i + 1, len(self.regions)):
                shared = self._shared_edge(i, j)
                if shared is None:
                    continue
                self._links.setdefault(i, {})[j] = shared

    def _shared_edge(self, i, j):
        other = set(frozenset(edge) for edge in polygon_edges(self.regions[j]))
        for edge in polygon_edges(self.regions[i]):
            if frozenset(edge) in other:
                return edge
        return None

    def neighbours(self, index):
        """Region indices that share an edge with the given region."""
        return sorted(self._links.get(index, {}))

    def portal(self, i, j):
        """The edge shared by two regions."""
        return self._links[i][j]

    def locate(self, point):
        """Index of the region that contains the point, or None."""
        for index, region in enumerate(self.regions):
            if point_in_region(region, point):
                return index
        return None

    def find_region_path(self, start, goal):
        """Region indices from the region holding start to the one holding goal."""
        origin = self.locate(start)
        target = self.locate(goal)
        if origin is None:
            raise ValueError("start point is outside the mesh: %r" % (start,))
        if target is None:
            raise ValueError("goal point is outside the mesh: %r" % (goal,))
        if origin == target:
            return [origin]
        queue = [origin]
        came_from = {origin: None}
        while queue:
            node = queue.pop(0)
            for neighbour in self.neighbours(node):
                if neighbour in came_from:
                    continue
                came_from[neighbour] = node
                if neighbour == target:
                    return self._chain(came_from, node)
                queue.append(neighbour)
        return None

    def _chain(self, came_from, node):
        chain = []
        while node is not None:
            chain.append(node)
            node = came_from[node]
        chain.reverse()
        return chain

    def find_path(self, start, goal):
        """Waypoints of the smoothed route, or None when there is none."""
        start = (float(start[0]), float(start[1]))
        goal = (float(goal[0]), float(goal[1]))
        regions = self.find_region_path(start, goal)
        if regions is None:
            return [start]
        portals = [
            self.portal(regions[i], regions[i + 1]) for i in range(len(regions) - 1)
        ]
        return simplify(funnel(start, goal, portals))


def funnel(start, goal, portals):
    """Waypoints of a short route from start to goal through the given portals.

    Every portal is the pair of points that bound the opening between two
    neighbouring regions.
    """
    points = [start]
    apex = start
    left = None
    right = None
    left_index = -1
    right_index = -1
    index = 0
    total = len(portals)
    while index < total:
        candidate_left, candidate_right = portals[index]
        if left is None:
            left = candidate_left
            right = candidate_right
            left_index = index
            right_index = index
            index += 1
            continue
        if candidate_right != apex and (
            right == apex or _area(apex, right, candidate_right) <= 0.0
        ):
            if (
                left == apex
                or right == apex
                or _area(apex, left, candidate_right) > 0.0
            ):
                right = candidate_right
                right_index = index
            else:
                points.append(left)
                apex = left
                left = apex
                right = apex
                index = left_index + 1
                continue
        if candidate_left != apex and (
            left == apex or _area(apex, left, candidate_left) >= 0.0
        ):
            if (
                right == apex
                or left == apex
                or _area(apex, right, candidate_left) < 0.0
            ):
                left = candidate_left
                left_index = index
            else:
                points.append(right)
                apex = right
                left = apex
                right = apex
                index = right_index + 1
                continue
        index += 1
    points.append(goal)
    return points


def simplify(points, epsilon=EPSILON):
    """Drop duplicate and needless waypoints from a route."""
    if points is None:
        return None
    cleaned = []
    for point in points:
        if cleaned and _distance(cleaned[-1], point) <= epsilon:
            continue
        cleaned.append((float(point[0]), float(point[1])))
    result = []
    for point in cleaned:
        while len(result) >= 2 and _is_collinear(result[-2], result[-1], point):
            result.pop()
        result.append(point)
    return result


def path_length(points):
    """Total length of a waypoint chain; None for a missing route."""
    if points is None:
        return None
    total = 0.0
    for first, second in zip(points, points[1:]):
        total += abs(second[0] - first[0]) + abs(second[1] - first[1])
    return total
