from itertools import chain, product, cycle
from functools import reduce
from shapely_extension import *
from rtree_set import RtreeSet
from spatial_graph import SpatialGraph
import networkx as nx


class ObstacleMetric:
    def __init__(self, surface, obstacles):
        self.surface = surface
        self.obstacles = RtreeSet(obstacles)
        self.points = [point for polygon in self.polygons() for point in polygon.points()]
        self.point_visibility_polygons = None
        self.visibility_polygons = None
        self.polygon_visibility_points = None
        self.visibility_graph = None
        self.diameter = self.calc_diameter()
        self.bounding_box = Polygon.from_bounds(*surface.bounds).exterior

    def polygons(self):
        return chain([self.surface], self.obstacles)

    def segments(self):
        return (segment for polygon in self.polygons() for segment in polygon.segments())

    def calc_diameter(self):
        left, bottom, right, top = self.surface.bounds
        return np.linalg.norm(np.array([right, top]) - np.array([left, bottom]))

    def build_point_visibility_polygons(self):
        self.point_visibility_polygons = dict()
        for point in self.points:
            self.point_visibility_polygons[point] = self.calc_visibility_polygon(point)

    def calc_visibility_polygon(self, point, radius=None):
        visibility_polygon = self.surface
        if radius is not None:
            visibility_polygon = visibility_polygon.intersection(point.buffer(radius))
        for obstacle in self.obstacles:
            visibility_polygon = visibility_polygon.difference(obstacle)
        for segment in self.segments():
            if segment.intersects(point):
                continue
            shadow_polygon = self.calc_shadow_polygon(point, segment)
            visibility_polygon = visibility_polygon.difference(shadow_polygon)
        return visibility_polygon

    def calc_shadow_polygon(self, point, segment):
        point0, point1 = segment.points()
        if not LinearRing.from_points([point, point0, point1]).is_ccw:
            point0, point1 = point1, point0

        vector0 = point0 - point
        continuity_direction0 = vector0 / np.linalg.norm(vector0)
        continuity0 = LineString([point0, point0 + self.diameter * continuity_direction0])
        bound_intersection0 = continuity0.intersection(self.bounding_box)

        vector1 = point1 - point
        continuity_direction1 = vector1 / np.linalg.norm(vector1)
        continuity1 = LineString([point1, point1 + self.diameter * continuity_direction1])
        bound_intersection1 = continuity1.intersection(self.bounding_box)

        shadow_polygon_points = [point0]
        if bound_intersection0 != shadow_polygon_points[-1]:
            shadow_polygon_points.append(bound_intersection0)

        segment_iterator = iter(cycle(reversed(self.bounding_box.segments())))
        segment = EmptyGeometry()
        while not segment.intersects(bound_intersection0):
            segment = next(segment_iterator)

        while not segment.intersects(bound_intersection1):
            if segment.points()[0] != shadow_polygon_points[-1]:
                shadow_polygon_points.append(segment.points()[0])
            segment = next(segment_iterator)
        if bound_intersection1 != shadow_polygon_points[-1]:
            shadow_polygon_points.append(bound_intersection1)
        if point1 != shadow_polygon_points[-1]:
            shadow_polygon_points.append(point1)

        if len(shadow_polygon_points) > 2:
            return Polygon.from_points(shadow_polygon_points)
        else:
            return EmptyGeometry()

    def build_visibility_subdivision(self):
        if not self.point_visibility_polygons:
            self.build_point_visibility_polygons()

        self.visibility_polygons = RtreeSet()
        self.polygon_visibility_points = dict()
        self.recursive_intersect(set(), self.surface, 0)

    def recursive_intersect(self, points, current_intersection, n):
        for i in range(n, len(self.points)):
            point = self.points[i]
            visible_polygon = self.point_visibility_polygons[point]
            intersection = current_intersection.intersection(visible_polygon)
            if intersection.is_multipart_geometry():
                intersection = reduce(BaseGeometry.union, filter(lambda shape: type(shape) is Polygon, intersection), EmptyGeometry())
            if type(intersection) in [Polygon, MultiPolygon]:
                self.recursive_intersect(points | {point}, intersection, i + 1)

                difference = current_intersection.difference(intersection)
                if difference.is_multipart_geometry():
                    difference = reduce(BaseGeometry.union, filter(lambda shape: type(shape) is Polygon, difference), EmptyGeometry())
                if type(difference) in [Polygon, MultiPolygon]:
                    self.recursive_intersect(points, difference, i + 1)

                break
        else:
            if points:
                polygons = [current_intersection] if type(current_intersection) is Polygon else current_intersection
                for polygon in polygons:
                    if type(polygon) is Polygon:
                        self.polygon_visibility_points[polygon] = points
                        self.visibility_polygons.insert(polygon)

    def build_visibility_graph(self):
        # TODO: use visibility_polygons - foreach 2 points, check if the connecting segment is within visibility polygon
        self.visibility_graph = SpatialGraph()

        for polygon in chain([self.surface], self.obstacles):
            for point in polygon.points():
                self.visibility_graph.add_node(point)

        for polygon in chain([self.surface], self.obstacles):
            for segment in polygon.segments():
                self.visibility_graph.add_edge(*segment.points())

        for polygon0, polygon1 in product(chain([self.surface], self.obstacles), repeat=2):
            for point0 in polygon0.points():
                segments = polygon1.segments()
                segments.append(segments[0])
                for segment0, segment1 in zip(segments[:-1], segments[1:]):
                    point1 = segment1[0]
                    if point0 == point1:
                        continue
                    if self.is_visible(point0, point1):
                        self.visibility_graph.add_edge(point0, point1)

    def is_visible(self, point0, point1):
        segment = LineString((point0, point1))
        if not self.surface.contains(segment):
            return False
        for polygon in self.obstacles.intersection(segment):
            if segment.intersection(polygon) not in {point0, point1, MultiPoint([point0, point1])}:
                # print(segment.intersection(polygon))
                return False
        return True

    def get_containing_polygon(self, point):
        for polygon in self.visibility_polygons.intersection(point):
            return polygon

    def get_visible_points(self, point):
        polygon = self.get_containing_polygon(point)
        if polygon:
            points = self.polygon_visibility_points[polygon]
            return points

    def get_shortest_path(self, point0, point1):
        if self.is_visible(point0, point1):
            return [point0, point1]
        else:
            self.visibility_graph.add_node(point0)
            self.visibility_graph.add_node(point1)
            visible_points0 = self.get_visible_points(point0)
            visible_points1 = self.get_visible_points(point1)
            if visible_points0 and visible_points1:
                for visible_point0 in visible_points0:
                    self.visibility_graph.add_edge(point0, visible_point0)
                for visible_point1 in visible_points1:
                    self.visibility_graph.add_edge(point1, visible_point1)

                # TODO: use depth search with euclidean distance as deceision factor
                shortest_path = nx.dijkstra_path(self.visibility_graph, point0, point1)
                return shortest_path

            self.visibility_graph.remove_node(point0)
            self.visibility_graph.remove_node(point1)

    def is_within_radius(self, point0, point1, distance):
        ret_val = None
        if point0.distance(point1) > distance:
            ret_val = False
        elif self.is_visible(point0, point1):
            ret_val = True
        else:
            self.visibility_graph.add_node(point0)
            self.visibility_graph.add_node(point1)
            visible_points0 = self.get_visible_points(point0)
            visible_points1 = self.get_visible_points(point1)
            if visible_points0 and visible_points1:
                # TODO: replace by add_edges_from(...)
                for visible_point0 in visible_points0:
                    self.visibility_graph.add_edge(point0, visible_point0)
                for visible_point1 in visible_points1:
                    self.visibility_graph.add_edge(point1, visible_point1)

                shortest_path_length = nx.single_source_dijkstra_path_length(self.visibility_graph, point0, point1, cutoff=distance)
                ret_val = shortest_path_length < distance

            self.visibility_graph.remove_node(point0)
            self.visibility_graph.remove_node(point1)

        return ret_val

    def calc_obstacle_metric_distance_polygon(self, point):
        # TODO
        return
