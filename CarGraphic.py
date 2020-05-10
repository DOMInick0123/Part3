import math
import pyglet
import Car


class CarGraphic(pyglet.sprite.Sprite, Car.Car):
    img = None

    def __init__(self, width, height,  network=None):
        Car.Car.__init__(self, network)
        if self.img is None:
            self.img = pyglet.image.load('CarSmall.png')
            self.img.anchor_x = int((Car.rear_wing + Car.c) * 2.5)
            self.img.anchor_y = int(Car.width * 2.5 / 2)
        pyglet.sprite.Sprite.__init__(self, self.img, x=width//2, y=height//2)
        self.rotation = 0

    def update_car(self, dt=0.01):
        Car.Car.update_car(self, dt)
        self.rotation = math.degrees(self.rot_rad)
