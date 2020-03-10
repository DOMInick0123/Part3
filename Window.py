import math

import pyglet
from pyglet.window import key
from Player import Player
import numpy as np
from PIL import Image

network = np.load('bp.npy', allow_pickle=True)
#network = [([[-0.0001, 0., 1., 1., 1., 0.], [0., -0.3, -0.5, -1., -0.5, -0.3], [0., 0.1, 0.3, 0., -0.3, -0.1]], [1., -1., 0.])]
player = Player(network)
label = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=50, y=700,
                          anchor_x='center', anchor_y='center', color=(0, 255, 255, 255))
left = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=50, y=600,
                          anchor_x='center', anchor_y='center', color=(0, 255, 255, 255))
right = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=50, y=500,
                          anchor_x='center', anchor_y='center', color=(0, 255, 255, 255))
track = pyglet.sprite.Sprite(pyglet.image.load('test_track.jpg'))

if __name__ == '__main__':
    window = pyglet.window.Window(width=1920, height=1080, caption="Car")
    pyglet.gl.glClearColor(1., 1., 1., 1.)

    @window.event
    def on_draw():
        window.clear()
        track.draw()
        player.draw()
        label.draw()
        left.draw()
        right.draw()


    def update(dt):
        if player.counter == 5:
            player.counter = 0
            player.think()
        player.update_player()
        track.x = 960-player.pos_x
        track.y = 540-player.pos_y
        label.text = str(player.alive)
        left.text = str(round(player.score, 2))
        right.text = str(round(player.velocity_local_x, 2))

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


    pyglet.clock.schedule_interval(update, 0.01)
    pyglet.app.run()
