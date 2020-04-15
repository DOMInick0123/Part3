import pyglet
from pyglet.window import key
from CarGraphic import CarGraphic
import numpy as np

width = 1920
height = 1080
networks = np.load('grid.npy', allow_pickle=True)
bs = 0.
nn = []
for network in networks:
    if network[7] > bs:
        bs = network[7]
        nn = network[0]
print(bs)
player = CarGraphic(width, height, nn)
#player = CarGraphic(width, height, np.load('error.npy', allow_pickle=True))

right = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=50, y=500,
                          anchor_x='center', anchor_y='center', color=(0, 255, 255, 255))
gear = pyglet.text.Label('Hello, world',
                         font_name='Times New Roman',
                         font_size=20,
                         x=50, y=400,
                         anchor_x='center', anchor_y='center', color=(0, 255, 255, 255))
rpm = pyglet.text.Label('Hello, world',
                         font_name='Times New Roman',
                         font_size=20,
                         x=50, y=300,
                         anchor_x='center', anchor_y='center', color=(0, 255, 255, 255))
#track = pyglet.sprite.Sprite(pyglet.image.load('test_track.jpg'))
#track = pyglet.sprite.Sprite(pyglet.image.load('track_graphics.jpg'))
#track = pyglet.sprite.Sprite(pyglet.image.load('monza.jpg'))
track = pyglet.sprite.Sprite(pyglet.image.load('testtrack.jpg'))
scale = 2.5
track.scale = scale

if __name__ == '__main__':
    window = pyglet.window.Window(width=width, height=height, caption="Car")
    pyglet.gl.glClearColor(1., 1., 1., 1.)

    @window.event
    def on_draw():
        window.clear()
        track.draw()
        player.draw()
        right.draw()
        gear.draw()
        rpm.draw()


    def update(dt):
        player.think()
        for _ in range(5):
            player.update_car()
        track.x = 960-(player.pos_x * scale)
        track.y = 540-(player.pos_y * scale)
        right.text = str(round(player.velocity_local_x*3.6, 2))
        gear.text = str(player.alive_counter)
        rpm.text = str(round(dt, 4))

    @window.event
    def on_key_press(symbol, modifiers):
        if symbol == key.W:
            player.acceleration_pedal = 1
        elif symbol == key.S:
            player.braking_pedal = 1
        elif symbol == key.A:
            player.steering = -1
        elif symbol == key.D:
            player.steering = 1
        elif symbol == key.PAGEUP:
            player.gear += 1
            player.gear = min(5, player.gear)
        elif symbol == key.PAGEDOWN:
            player.gear -= 1
            player.gear = max(0, player.gear)

    @window.event
    def on_key_release(symbol, modifiers):
        if symbol == key.W:
            player.acceleration_pedal = 0
        elif symbol == key.S:
            player.braking_pedal = 0
        elif symbol == key.A:
            player.steering = 0
        elif symbol == key.D:
            player.steering = 0


    pyglet.clock.schedule_interval(update, 0.05)
    pyglet.app.run()
