from rtree import index


class RtreeSet:
    """
    This class is a merge of the python built-in type 'set' and rtree 'Index' class.
    It gives a set for geometric objects, in addition to the regular set functionality,
    some functionality that make use of rtree indexing, like intersections check, nearest neighbour etc.
    It designed to work efficiently with shapely.geometry objects (Point, LineString, Polygon, ...).
    """
    def __init__(self, items=None):
        self.items = dict()
        self.index = index.Index(properties=index.Property())
        if items:
            for item in items:
                self.insert(item)

    def __bool__(self):
        return bool(self.items)

    def __contains__(self, item):
        return id(item) in self.items

    def __iter__(self):
        return iter(self.items.values())

    def __len__(self):
        return len(self.items)

    def __reduce__(self):
        return RtreeSet, (list(self.items.values()),)

    def __str__(self):
        return str({str(item) for item in self})

    def delete(self, item):
        self.index.delete(id(item), item.bounds)
        del self.items[id(item)]

    def insert(self, item):
        if id(item) in self.items:
            raise Exception()
        else:
            self.items[id(item)] = item
            self.index.insert(id(item), item.bounds)

    def intersection(self, item, exclude_item=True):
        for idx in self.index.intersection(item.bounds):
            other = self.items[idx]
            if not (exclude_item and item is other) and item.intersects(other):
                yield(other)

    def nearest(self, item, exclude_item=True):
        if not exclude_item and id(item) in self.items:
            return item
        for idx in self.index.nearest(item.bounds, 2):
            if self.items[idx] is not item:
                nearest_item = self.items[idx]
                break
        else:
            return
        search_radius = item.distance(nearest_item) * 1.01
        buffer = item.buffer(search_radius)
        return min(filter(lambda other: other is not item, self.intersection(buffer)),
                   key=lambda other: item.distance(other))
