# geometry

This package includes some utils to be used in the field of computational geometry.

*SpatialGraph*, which inherits from *networkx.Graph*, comes to mudel a special case of graph where the nodes are points in the plane and edges are the segments connecting them. The edge weight is hence automatically determined by the euclidian distance between the nodes it connects.

*RtreeSet* gives an abstraction to the common module of *Rtree*. It enables indexing of *shapely.geometry* objects (*Point*, *LineString*, *Polygon*, ...) simply by putting them in a set-like collection. The *insert*/*delete* operations doesn't require any information but the shape itself. Two geometric methods are implemented: *intersection(item)* which returns a generator of items in the collection that intersects the given item, and *nearest(item, n=1)* which returns the *n* nearest items in the collection to the given item.

*RtreeDict* works similarly to *RtreeSet* except that each item in the collection has also a value that can be of each type.

*ObstacleMetric* gives an efficient way (with very heavy preprocessing) to calculate shortest path between points in a surface with obstacle. It is based on the fact that the shortest path is either the euclidian path or it goes through the nodes (corners) of the obstacles. The algorighm is working as follows:
1. For each (convex) node, calculate its *visibility polygon* (the area which is visible from that point).
2. Intersect all visibility polygons recursively to create a *plannar subdivision* of polygons and a map between polygon to the nodes it sees. 
3. create a *visibility graph* between all nodes.
Distance calculation between two given points (which are not in line of sight) is done by finding the visible nodes of each point, add each point with its visible nodes to the visibility graph, and run *dijkstra* to find the shortest path.
