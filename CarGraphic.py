import math
import pyglet
from Car import Car


class CarGraphic(pyglet.sprite.Sprite, Car):
    img = None

    def __init__(self, width, height,  network=None):
        Car.__init__(self, network)
        if self.img is None:
            self.img = pyglet.image.load('CarSmall.png')
            self.img.anchor_x = self.anch_x
            self.img.anchor_y = self.anch_y
        pyglet.sprite.Sprite.__init__(self, self.img, x=width//2, y=height//2)
        self.rotation = 0

    def update_car(self, dt=0.01):
        Car.update_car(self, dt)
        self.rotation = math.degrees(self.rot_rad)
