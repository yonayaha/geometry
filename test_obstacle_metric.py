from itertools import chain, product, cycle
import random
from shapely_extension import *
import PIL.Image as Image
import PIL.ImageDraw as ImageDraw

from obstacle_metric import ObstacleMetric


class TestObstacleMetric:
    def __init__(self, obstacle_metric):
        self.obstacle_metric = obstacle_metric

    def obstacle_image_draw(self):
        image = Image.new("RGB", (600, 600))
        draw = ImageDraw.Draw(image)
        self.obstacle_metric.circum.draw(draw, fill=(255, 255, 255, 255))
        for obstacle in self.obstacle_metric.obstacles:
            obstacle.draw(draw, fill=(0, 0, 0, 255))

        return image, draw

    def show_obstacle_plane(self):
        image, draw = self.obstacle_image_draw()
        image.show()

    def show_visibility_polygons(self):
        if not self.obstacle_metric.point_visibility_polygons:
            return
        for point in self.obstacle_metric.points:
            visibility_polygon = self.obstacle_metric.point_visibility_polygons[point]
            image, draw = self.obstacle_image_draw()

            visibility_polygon.draw(draw, fill=(255, 0, 0, 255))
            # for interior in visibility_polygon.interiors:
            #     draw.polygon(tuple(zip(*interior.xy)), fill=(255,255,255,255))

            point.draw(draw, radius=3, fill=(0,255,0,255))

            image.show()

    def show_visibility_subdivision(self):
        image, draw = self.obstacle_image_draw()
        for i, polygon in enumerate(self.obstacle_metric.polygon_visibility_points):
            fill = tuple([*([random.randint(0, 256)] * 3), 255])
            polygon.draw(draw, fill=fill)
        image.show()

    def show_visibility_graph(self):
        image, draw = self.obstacle_image_draw()

        for point in self.obstacle_metric.visibility_graph.nodes:
            draw.point(tuple(zip(*point.xy)))
        for point0, point1 in self.obstacle_metric.visibility_graph.edges:
            draw.line(tuple(chain(zip(*point0.xy), zip(*point1.xy))), fill=(0,0,0,255))
        image.show()

    def test_get_containing_polygon(self):
        while True:
            point = Point(500 * random.random(), 500 * random.random())
            if self.obstacle_metric.circum.contains(point) and not any(map(
                    lambda obstacle: obstacle.contains(point), self.obstacle_metric.obstacles)):
                polygon = self.obstacle_metric.get_containing_polygon(point)

                image, draw = self.obstacle_image_draw()

                polygon.draw(draw, fill=(255, 0, 0, 255))
                point.draw(draw, radius=3, fill=(0, 255, 0, 255))

                image.show()
                break

    def test_get_visible_points(self):
        while True:
            point = Point(500 * random.random(), 500 * random.random())
            visible_points = self.obstacle_metric.get_visible_points(point)
            if visible_points:
                image, draw = self.obstacle_image_draw()

                point.draw(draw, radius=3, fill=(0, 255, 0, 255))
                for visible_point in visible_points:
                    visible_point.draw(draw, radius=3, fill=(255, 0, 0, 255))
                image.show()
                break

    def test_get_shortest_path(self):
        while True:
            point0 = Point(500 * random.random(), 500 * random.random())
            point1 = Point(500 * random.random(), 500 * random.random())
            shortest_path = self.obstacle_metric.get_shortest_path(point0, point1)
            if not shortest_path:
                continue

            image, draw = self.obstacle_image_draw()

            point0.draw(draw, radius=3, fill=(0, 255, 0, 255))
            point1.draw(draw, radius=3, fill=(0, 255, 0, 255))

            for mid_point0, mid_point1 in zip(shortest_path[:-1], shortest_path[1:]):
                draw.line(tuple(chain(zip(*mid_point0.xy), zip(*mid_point1.xy))), fill=(0,0,0,255))

            image.show()
            break


circum = Polygon([[0,0], [0,500], [500,500], [500,0]])
obstacles = [
    # Polygon([[100,100],[100,400],[400,400],[400,100],[300,100],[300,300],[200,300],[200,100]])
    Polygon([[100,100],[100,400],[200,400],[200,100]]),
    Polygon([[300,100],[300,300],[400,300],[400,100]])
]

om = ObstacleMetric(circum, obstacles)
tom = TestObstacleMetric(om)
# tom.show_obstacle_plane()
om.build_point_visibility_polygons()
# tom.show_visibility_polygons()
om.build_visibility_subdivision()
# tom.show_visibility_subdivision()
om.build_visibility_graph()
# tom.show_visibility_graph()

# while True: tom.test_get_containing_polygon(); input()
# while True: tom.test_get_visible_points(); input()
while True: tom.test_get_shortest_path(); input()
