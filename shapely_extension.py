from itertools import islice
from shapely.geometry import *
from shapely.geometry.base import *


BaseGeometry.__bool__ = lambda self: not self.is_empty
BaseGeometry.is_multipart_geometry = lambda self: issubclass(type(self), BaseMultipartGeometry)

Point.__add__ = lambda self, vector: Point(np.array(self) + vector)
Point.__sub__ = lambda self, other: np.array(self) - np.array(other)
Point.__hash__ = lambda self: hash(tuple(zip(*self.xy)))
Point.draw = lambda self, draw, radius, fill: draw.ellipse(self.buffer(radius).bounds, fill=fill)

LineString.__hash__ = lambda self: hash(tuple(zip(*self.xy)))
LineString.points = lambda self: [Point(xy) for xy in zip(*self.xy)]
LineString.__getitem__ = lambda self, idx: Point(coord[idx] for coord in self.xy)
LineString.__mul__ = lambda self, factor: LineString(
    [(self[0] + 2 * (point - self[0])) for point in self.points()])
LineString.__add__ = lambda self, point0: LineString([point + np.array(point0) for point in self.points()])  # keep order: line + point
LineString.segments = lambda self: [LineString((xy0, xy1)) for xy0, xy1 in
                                    zip(islice(zip(*self.xy), len(self.xy[0]) - 1), islice(zip(*self.xy), 1, None))]

LinearRing.points = lambda self: [Point(xy) for xy in zip(*self.xy)][:-1]
LinearRing.from_points = lambda point_list: LinearRing([np.array(point) for point in point_list])

Polygon.__hash__ = lambda self: hash(tuple(zip(*self.exterior.xy), *(tuple(zip(*interior.xy)) for interior in self.interiors)))
Polygon.points = lambda self: self.exterior.points() if self.exterior else []
Polygon.segments = lambda self: self.exterior.segments() if self.exterior else []
Polygon.from_points = lambda point_list: Polygon([np.array(point) for point in point_list])
Polygon.draw = lambda self, draw, fill: draw.polygon(tuple(zip(*self.exterior.xy)), fill=fill)

BaseMultipartGeometry.__hash__ = lambda self: hash(tuple(self))
