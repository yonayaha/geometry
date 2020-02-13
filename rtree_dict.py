from rtree_set import RtreeSet


class RtreeDict(RtreeSet):
    def __delitem__(self, key):
        self.delete(key)

    def __getitem__(self, key):
        return self._items[id(key)][1]

    def __iter__(self):
        return iter(item[0] for item in self._items.values())

    def __reduce__(self):
        return RtreeDict, (list(self._items.values()),)

    def __setitem__(self, key, value):
        if key not in self:
            self.index.insert(id(key), key.bounds)
        self._items[id(key)] = (key, value)

    def __str__(self):
        return str({str(key): str(value) for key, value in self.items()})

    def insert(self, item):
        self.__setitem__(*item)

    def intersection(self, item):
        if item.bounds:
            for idx in self.index.intersection(item.bounds):
                other = self._items[idx][0]
                if item.intersects(other):
                    yield(other)

    def items(self):
        return self._items.values()

    def keys(self):
        return (item[0] for item in self._items.values())

    def nearest(self, item, n=1):
        if not item.bounds:
            return []
        item_list = list(self.index.nearest(item.bounds, n))
        search_radius = max(self._items[idx][0].distance(item) for idx in item_list) * 1.01
        if search_radius != 0:
            buffer = item.buffer(search_radius)  # TODO: can do better
            item_list = sorted(self.intersection(buffer), key=lambda other: item.distance(other))
        return item_list[:n]

    def values(self):
        return (item[1] for item in self._items.values())
