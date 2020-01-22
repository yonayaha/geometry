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
        self.visibility_polygons = None
        self.polygon_visibility_points = None
        self.visibility_graph = None
        self.diameter = self.calc_diameter()
        self.bounding_box = self.calc_bounding_box()

    def polygons(self):
        return chain([self.surface], self.obstacles)

    def segments(self):
        return (segment for polygon in self.polygons() for segment in polygon.segments())

    def calc_diameter(self):
        left, bottom, right, top = self.surface.bounds
        return np.linalg.norm(np.array([right, top]) - np.array([left, bottom]))

    def calc_bounding_box(self):
         union = reduce(BaseGeometry.union, self.obstacles, self.surface)
         return Polygon.from_bounds(*union.buffer(1).bounds).exterior

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
        print('building point visibility polygons')
        point_visibility_polygons = dict()
        for point in self.points:
            point_visibility_polygons[point] = self.calc_visibility_polygon(point)

        print('building visibility subdivision')
        self.visibility_polygons = RtreeSet()
        self.polygon_visibility_points = dict()
        subdivision = {self.surface: set()}
        print(len(self.points))
        for i, point in enumerate(self.points):
            print(i, len(subdivision))
            new_subdivision = dict()
            visible_polygon = point_visibility_polygons[point]
            for polygon, points in subdivision.items():
                try:
                    intersection = polygon.intersection(visible_polygon)
                    if type(intersection) is Polygon and intersection.area > 0.001:
                        new_subdivision[intersection] = points | {point}
                    elif intersection.is_multipart_geometry():
                        for part in intersection:
                            if type(part) is Polygon and part.area > 0.001:
                                new_subdivision[part] = points | {point}
                except Exception as ex:
                    print(ex)
                    print(polygon.area, visible_polygon.area)

                try:
                    difference = polygon.difference(visible_polygon)
                    if type(difference) is Polygon and difference.area > 0.001:
                        new_subdivision[difference] = points
                    elif difference.is_multipart_geometry():
                        for part in difference:
                            if type(part) is Polygon and part.area > 0.001:
                                new_subdivision[part] = points
                except Exception as ex:
                    print(ex)
                    print(polygon.area, visible_polygon.area)

            subdivision = new_subdivision

        for polygon, points in subdivision.items():
            if points:
                self.polygon_visibility_points[polygon] = points
                self.visibility_polygons.insert(polygon)

    def build_visibility_graph(self):
        print('building visibility graph')
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
        return set()

    def add_visibility_edges(self, point):
        self.visibility_graph.add_node(point)
        visible_points = self.get_visible_points(point)
        self.visibility_graph.add_edges_from((point, visible_point) for visible_point in visible_points)
        return bool(visible_points)

    def get_shortest_path(self, point0, point1):
        shortest_path = None
        if self.is_visible(point0, point1):
            shortest_path = [point0, point1]
        else:
            is_visible0 = self.add_visibility_edges(point0)
            is_visible1 = self.add_visibility_edges(point1)
            if is_visible0 and is_visible1:
                # TODO: use depth search with euclidean distance as deceision factor
                shortest_path = nx.dijkstra_path(self.visibility_graph, point0, point1)
            # TODO: what if the node is already part of the graph
            self.visibility_graph.remove_nodes_from([point0, point1])

        return shortest_path

    def get_point_distance_machine(self, point, cutoff=None):
        point_distance_machine = None
        if self.add_visibility_edges(point):
            path_legths = nx.single_source_dijkstra_path_length(self.visibility_graph, point, cutoff)

            def point_distance_machine(point1):
                if self.is_visible(point, point1):
                    return point.distance(point1)
                visible_points = [point2 for point2 in (self.get_visible_points(point1) or []) if point2 in path_legths]
                if visible_points:
                    return min(point1.distance(point2) + path_legths[point2] for point2 in visible_points)

        self.visibility_graph.remove_node(point)
        return point_distance_machine

    def get_point_path_machine(self, point, cutoff=None):
        point_path_machine = None
        if self.add_visibility_edges(point):
            path_legths, paths = nx.single_source_dijkstra(self.visibility_graph, point, None, cutoff)

            def point_path_machine(point1):
                if self.is_visible(point, point1):
                    return [point, point1]
                visible_points = [point2 for point2 in (self.get_visible_points(point1) or []) if point2 in path_legths]
                if visible_points:
                    return min((point1.distance(point2) + path_legths[point2], paths[point2]) for point2 in visible_points)[1] + [point1]

        self.visibility_graph.remove_node(point)
        return point_path_machine

    def is_within_radius(self, point0, point1, distance):
        ret_val = None
        if point0.distance(point1) > distance:
            ret_val = False
        elif self.is_visible(point0, point1):
            ret_val = True
        else:
            is_visible0 = self.add_visibility_edges(point0)
            is_visible1 = self.add_visibility_edges(point1)
            if is_visible0 and is_visible1:
                shortest_path_length = nx.single_source_dijkstra_path_length(self.visibility_graph, point0, point1, cutoff=distance)
                ret_val = shortest_path_length < distance
            self.visibility_graph.remove_nodes_from([point0, point1])

        return ret_val

    def calc_obstacle_metric_distance_polygon(self, point):
        # TODO
        return