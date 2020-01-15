import random
from shapely_extension import *
import pickle
import time

from obstacle_metric import ObstacleMetric
from surface_image import SurfaceImage


class TestObstacleMetric:
    def __init__(self, obstacle_metric):
        self.obstacle_metric = obstacle_metric

    def obstacle_surface_image(self):
        image = SurfaceImage(self.obstacle_metric.surface)
        for obstacle in self.obstacle_metric.obstacles:
            image.draw_polygon(obstacle, fill=(0, 0, 0, 255))

        return image

    def random_point(self):
        bounds = self.obstacle_metric.surface.bounds
        x = bounds[0] + random.random() * (bounds[2] - bounds[0])
        y = bounds[1] + random.random() * (bounds[3] - bounds[1])
        return Point(x, y)

    def show_obstacle_plane(self):
        image = self.obstacle_surface_image()
        image.show()

    def show_visibility_subdivision(self):
        image = self.obstacle_surface_image()
        for i, polygon in enumerate(self.obstacle_metric.polygon_visibility_points):
            fill = tuple([*([random.randint(0, 256)] * 3), 255])
            image.draw_polygon(polygon, fill=fill)
        image.show()

    def show_visibility_graph(self):
        image = self.obstacle_surface_image()

        for point in self.obstacle_metric.visibility_graph.nodes:
            image.draw_point(point, fill=(150, 150, 150, 255))
        for point0, point1 in self.obstacle_metric.visibility_graph.edges:
            image.draw_line(point0, point1, fill=(0, 0, 0, 255))
        image.show()

    def test_get_containing_polygon(self):
        while True:
            point = self.random_point()
            if self.obstacle_metric.surface.contains(point) and not any(map(
                    lambda obstacle: obstacle.contains(point), self.obstacle_metric.obstacles)):
                polygon = self.obstacle_metric.get_containing_polygon(point)

                image = self.obstacle_surface_image()

                image.draw_polygon(polygon, fill=(255, 0, 0, 255))
                image.draw_point(point, fill=(0, 255, 0, 255))

                image.show()
                break

    def test_get_visible_points(self):
        while True:
            point = self.random_point()
            visible_points = self.obstacle_metric.get_visible_points(point)
            if visible_points:
                image = self.obstacle_surface_image()

                image.draw_point(point, fill=(0, 255, 0, 255))
                for visible_point in visible_points:
                    image.draw_point(visible_point, fill=(255, 0, 0, 255))
                image.show()
                break

    def test_get_shortest_path(self):
        while True:
            point0 = self.random_point()
            point1 = self.random_point()
            t0 = time.time()
            shortest_path = self.obstacle_metric.get_shortest_path(point0, point1)
            if not shortest_path:
                continue
            print('total:', time.time() - t0)
            image = self.obstacle_surface_image()

            image.draw_point(point0, fill=(0, 255, 0, 255))
            image.draw_point(point1, fill=(0, 255, 0, 255))

            for mid_point0, mid_point1 in zip(shortest_path[:-1], shortest_path[1:]):
                image.draw_line(mid_point0, mid_point1, fill=(0, 0, 0, 255))

            image.show()
            break

    def test_get_point_distance_machine(self):
        while True:
            point = self.random_point()
            t0 = time.time()
            point_distance_machine = self.obstacle_metric.get_point_distance_machine(point, cutoff=None)
            t1 = time.time() - t0
            if not point_distance_machine:
                continue
            while True:
                point1 = self.random_point()
                t0 = time.time()
                distance = point_distance_machine(point1)
                if distance:
                    print(t1, time.time() - t0, distance)
                    break
            break

    def test_get_point_path_machine(self):
        while True:
            point = self.random_point()
            t0 = time.time()
            point_path_machine = self.obstacle_metric.get_point_path_machine(point, cutoff=None)
            t1 = time.time() - t0
            if not point_path_machine:
                continue
            while True:
                point1 = self.random_point()
                t0 = time.time()
                shortest_path = point_path_machine(point1)
                if shortest_path:
                    shortest_path.append(point1)
                    print(t1, time.time() - t0)
                    image = self.obstacle_surface_image()

                    image.draw_point(point, fill=(0, 255, 0, 255))
                    image.draw_point(point1, fill=(0, 255, 0, 255))

                    for mid_point0, mid_point1 in zip(shortest_path[:-1], shortest_path[1:]):
                        image.draw_line(mid_point0, mid_point1, fill=(0, 0, 0, 255))
                    image.show()
                    input()

surface = Polygon([[0,0], [0,500], [500,500], [500,0]])
obstacles = [
    # Polygon([[100,100],[100,400],[400,400],[400,100],[300,100],[300,300],[200,300],[200,100]])
    Polygon([[100,100],[100,400],[200,400],[200,100]]),
    Polygon([[300,100],[300,300],[400,300],[400,100]])
]

om = ObstacleMetric(surface, obstacles)
om.build_visibility_subdivision()
om.build_visibility_graph()
tom = TestObstacleMetric(om)
tom.show_obstacle_plane()
tom.show_visibility_subdivision()
tom.show_visibility_graph()

# while True: tom.test_get_containing_polygon(); input()
# while True: tom.test_get_visible_points(); input()
while True: tom.test_get_shortest_path(); input()
# while True: tom.test_get_point_distance_machine(); input()
# tom.test_get_point_path_machine()
