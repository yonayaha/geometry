from itertools import chain, product, combinations, cycle, repeat
from functools import reduce
import networkx as nx
from scipy.spatial.distance import directed_hausdorff

from shapely_extension import *
from rtree_set import RtreeSet
from rtree_dict import RtreeDict
from spatial_graph import SpatialGraph


class ObstacleMetric:
    EPSILON = 1e-12

    def __init__(self, surface, obstacles):
        self.obstacle_surface = reduce(BaseGeometry.difference, obstacles, surface)
        if self.obstacle_surface.is_multipart_geometry():
            self.obstacle_surface = max(self.obstacle_surface, key=lambda part: part.area)
        self.polygon_visibility_points = None
        self.visibility_graph = None
        self.diameter = self.calc_diameter()
        self.bounding_box = self.calc_bounding_box()

    def polygons(self):
        return chain([self.obstacle_surface.exterior], self.obstacle_surface.interiors)

    def points(self):
        return (point for polygon in self.polygons() for point in polygon.points())

    def segments(self):
        return (segment for polygon in self.polygons() for segment in polygon.segments())

    def calc_diameter(self):
        left, bottom, right, top = self.obstacle_surface.bounds
        return np.linalg.norm(np.array([right, top]) - np.array([left, bottom]))

    def calc_bounding_box(self):
         return Polygon.from_bounds(*self.obstacle_surface.buffer(1).bounds).exterior

    def calc_visibility_polygon(self, point, radius=None):
        visibility_polygon = self.obstacle_surface
        if radius is not None:
            visibility_polygon = visibility_polygon.intersection(point.buffer(radius))
        for segment in self.segments():
            if not segment.intersects(point) and segment.intersects(visibility_polygon):

                point0, point1 = segment.points()
                vector = point1 - point0
                direction = vector / np.linalg.norm(vector)
                buffered_segment = LineString([point0 - self.EPSILON * direction, point1 + self.EPSILON * direction])
                shadow_polygon = self.calc_shadow_polygon(point, buffered_segment)
                if shadow_polygon.is_valid:
                    try:
                        shadow_diff = visibility_polygon.difference(shadow_polygon)
                        if shadow_diff.is_valid:
                            visibility_polygon = shadow_diff
                    except Exception as ex:
                        print(ex)

        if visibility_polygon.is_multipart_geometry() and len(visibility_polygon) > 0:
            visibility_polygon = max(visibility_polygon, key=lambda part: part.area)

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

    def build_visibility_subdivision(self, tol=0):
        print('building point visibility polygons')
        point_visibility_polygons = dict()
        for i, (polygon, sign) in enumerate(chain([(self.obstacle_surface.exterior, -1)],
                                                  zip(self.obstacle_surface.interiors, repeat(1)))):
            print(i, '/', len(self.obstacle_surface.interiors))
            points = polygon.points()
            if not polygon.is_ccw:
                points.reverse()
            for point0, point1, point2 in zip(*(points[i:] + points[:i] for i in range(3))):
                vector0 = point1 - point0
                vector1 = point2 - point1
                if np.sign(np.cross(vector0, vector1)) == sign:
                    visibility_polygon = self.calc_visibility_polygon(point1)
                    point_visibility_polygons[point1] = visibility_polygon

        print('building visibility subdivision')
        self.polygon_visibility_points = RtreeDict()
        self.polygon_visibility_points[self.obstacle_surface] = set()
        for i, (point, visible_polygon) in enumerate(point_visibility_polygons.items()):
            print(i, '/', len(point_visibility_polygons), ':', len(self.polygon_visibility_points))
            added_parts = []
            for polygon in list(self.polygon_visibility_points.intersection(visible_polygon)):
                points = self.polygon_visibility_points[polygon]

                if visible_polygon.contains(polygon):
                    self.polygon_visibility_points[polygon] = points | {point}
                else:
                    del self.polygon_visibility_points[polygon]
                    try:
                        intersection = polygon.intersection(visible_polygon)
                        added_parts.extend(self.update_polygon_visibility_points(intersection, points | {point}))
                    except Exception as ex:
                        print(ex)
                        print(polygon.area, visible_polygon.area)

                    try:
                        difference = polygon.difference(visible_polygon)
                        added_parts.extend(self.update_polygon_visibility_points(difference, points))
                    except Exception as ex:
                        print(ex)
                        print(polygon.area, visible_polygon.area)

            if tol:
                i = 0
                while i < len(added_parts):
                    polygon0 = added_parts[i]
                    i += 1
                    if polygon0 not in self.polygon_visibility_points:
                        continue
                    closest_polygon = None
                    min_thickness = float('inf')
                    for polygon1 in self.polygon_visibility_points.intersection(polygon0.buffer(self.EPSILON)):
                        if polygon1 is not polygon0 and polygon0.distance(polygon1) < self.EPSILON:
                            thickness = directed_hausdorff(polygon0.exterior.coords, polygon1.exterior.coords)[0]
                            if thickness < min_thickness:
                                closest_polygon = polygon1
                                min_thickness = thickness
                    if min_thickness < tol:
                        try:
                            union = closest_polygon.union(polygon0)
                            points = self.polygon_visibility_points[closest_polygon]

                            del self.polygon_visibility_points[polygon0]
                            del self.polygon_visibility_points[closest_polygon]

                            # added_parts.extend(self.update_polygon_visibility_points(union, points))  # TODO: fix
                            self.update_polygon_visibility_points(union, points)

                        except Exception as ex:
                            print(ex)
                            print(polygon0.area, closest_polygon.area)

    def update_polygon_visibility_points(self, polygon, points):
        parts = []
        if type(polygon) is Polygon and not polygon.is_empty:
            parts = [polygon]
        elif polygon.is_multipart_geometry():
            parts = [part for part in polygon if type(part) is Polygon and not part.is_empty]
        for part in parts:
            self.polygon_visibility_points[part] = points
        return parts

    def build_visibility_graph(self):
        print('building visibility graph')
        self.visibility_graph = SpatialGraph()
        # TODO: avoid concave points
        self.visibility_graph.add_nodes_from(self.points())
        self.visibility_graph.add_edges_from((segment.points() for segment in self.segments()))
        for point0, point1 in combinations(self.points(), 2):
            if self.is_visible(point0, point1):
                self.visibility_graph.add_edge(point0, point1)

    def is_visible(self, point0, point1):
        segment = LineString((point0, point1))
        return self.obstacle_surface.contains(segment)

    def get_containing_polygon(self, point):
        for polygon in self.polygon_visibility_points.intersection(point):
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
