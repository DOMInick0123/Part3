import pyglet
from pyglet.window import key
from Player import Player
import numpy as np

network = np.load('bp.npy', allow_pickle=True)
player = Player(network)
label = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=1800, y=1000,
                          anchor_x='center', anchor_y='center')
left = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=1800, y=900,
                          anchor_x='center', anchor_y='center')
right = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=1800, y=800,
                          anchor_x='center', anchor_y='center')
track_graphic = pyglet.image.load('track.jpeg')

if __name__ == '__main__':
    window = pyglet.window.Window(width=1920, height=1080, caption="Car")
    pyglet.gl.glClearColor(0.5, 0.5, 0.5, 0.5)

    @window.event
    def on_draw():
        window.clear()
        track_graphic.blit(0, 0)
        player.draw()
        label.draw()
        left.draw()
        #right.draw()


    def update(dt):
        if player.counter == 5:
            player.think()
        player.update_player(dt)
        label.text = str(int(player.score))
        left.text = str(player.alive)

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
