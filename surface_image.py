from itertools import chain
from shapely_extension import *
import PIL.Image as Image
import PIL.ImageDraw as ImageDraw


class SurfaceImage:
    MAX_WIDTH = 800
    MAX_HEIGHT = 600

    def __init__(self, surface):
        self.bounds = surface.bounds
        self.ratio = min(SurfaceImage.MAX_WIDTH / self.bounds[2] - self.bounds[0],
                         SurfaceImage.MAX_HEIGHT / self.bounds[3] - self.bounds[1])
        self.image = Image.new("RGB", (600, 600))
        self.draw = ImageDraw.Draw(self.image)
        self.draw_polygon(surface, fill=(255, 255, 255, 255))

    def coords_to_pixel(self, coords):
        return tuple(np.array(self.bounds[:2]) + self.ratio * (np.array(coords) - np.array(self.bounds[:2])))

    def draw_point(self, point, fill):
        x, y = self.coords_to_pixel(point)
        self.draw.ellipse([x-3, y-3, x+3, y+3], fill=fill)

    def draw_line(self, point0, point1, fill):
        self.draw.line(tuple(chain(map(self.coords_to_pixel, zip(*point0.xy)), map(self.coords_to_pixel, zip(*point1.xy)))), fill=(0, 0, 0, 255))

    def draw_polygon(self, polygon, fill):
        self.draw.polygon(tuple(map(self.coords_to_pixel, zip(*polygon.exterior.xy))), fill=fill)

    def show(self):
        self.image.show()
