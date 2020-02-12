from rtree import index


class RtreeSet:
    """
    This class is a merge of the python built-in type 'set' and rtree 'Index' class.
    It gives a set for geometric objects, in addition to the regular set functionality,
    some functionality that make use of rtree indexing, like intersections check, nearest neighbour etc.
    It designed to work efficiently with shapely.geometry objects (Point, LineString, Polygon, ...).
    """
    def __init__(self, items=None):
        self._items = dict()
        self.index = index.Index(properties=index.Property())
        if items:
            for item in items:
                self.insert(item)

    def __bool__(self):
        return bool(self._items)

    def __contains__(self, item):
        return id(item) in self._items

    def __iter__(self):
        return iter(self._items.values())

    def __len__(self):
        return len(self._items)

    def __reduce__(self):
        return RtreeSet, (list(self._items.values()),)

    def __str__(self):
        return str({str(item) for item in self})

    def delete(self, item):
        self.index.delete(id(item), item.bounds)
        del self._items[id(item)]

    def insert(self, item):
        if id(item) in self._items:
            raise Exception()
        else:
            self._items[id(item)] = item
            self.index.insert(id(item), item.bounds)

    def intersection(self, item):
        if item.bounds:
            for idx in self.index.intersection(item.bounds):
                other = self._items[idx]
                if item.intersects(other):
                    yield(other)

    def nearest(self, item, n=1):
        item_list = list(self.index.nearest(item.bounds, n))
        search_radius = max(self._items[idx].distance(item) for idx in item_list) * 1.01
        if search_radius != 0:
            buffer = item.buffer(search_radius)  # TODO: can do better
            item_list = sorted(self.intersection(buffer), key=lambda other: item.distance(other))
        return item_list[:n]
