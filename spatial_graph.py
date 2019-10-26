import networkx as nx

from rtree_set import RtreeSet
from shapely_extension import *


class SpatialGraph(nx.Graph):

    class Segment(LineString):
        def __init__(self, *points):
            LineString.__init__(self, points)
            self.points = points

    def __init__(self):
        super().__init__()
        self.points = RtreeSet()
        self.segments = RtreeSet()
        self.edge_seg_dict = dict()

    def add_node(self, point, **attr):
        super().add_node(point, **attr)
        self.points.insert(point)

    def add_nodes_from(self, points, **attr):
        for point in points:
            self.add_node(point, **attr)

    def add_edge(self, point0, point1, **attr):
        segment = SpatialGraph.Segment(point0, point1)
        if 'weight' not in attr:
            attr['weight'] = segment.length
        super().add_edge(point0, point1, **attr)
        self.segments.insert(segment)
        self.edge_seg_dict[point0, point1] = segment
        self.edge_seg_dict[point1, point0] = segment

    def add_edges_from(self, edges, **attr):
        for edge in edges:
            self.add_edge(edge, **attr)

    def remove_node(self, point):
        if point not in self.points:
            raise KeyError()
        self.points.delete(point)
        for point0 in self.neighbors(point):
            self.remove_edge(point, point0)
        super().remove_node(point)

    def remove_nodes_from(self, points):
        for point in points:
            self.remove_node(point)

    def remove_edge(self, point0, point1):
        segment = self.edge_seg_dict[(point0, point1)]
        self.segments.delete(segment)
        del self.edge_seg_dict[(point0, point1)]
        del self.edge_seg_dict[(point1, point0)]
        super().remove_edge(point0, point1)

    def remove_edges_from(self, edges):
        for point0, point1 in edges:
            self.remove_edge(point0, point1)

    def update(self, edges=None, points=None):
        if points:
            for point in points:
                self.add_node(point)
        if edges:
            for point0, point1 in edges:
                self.add_edge(point0, point1)

    def nearest_node(self, point):
        if self.points:
            return self.points.nearest(point)

    def nearest_edge(self, point):
        if self.segments:
            return self.segments.nearest(point).points

    def nearest_point(self, point):
        if self.segments:
            seg = self.segments.nearest(point)
            return seg.interpolate(seg.project(point))

# TODO calc distance in graph metric
# TODO: create object of point on the graph (1d)
# TODO: r-ball in graph metric
# TODO: shortest path using depth search with euclidean distance as deceision factor
