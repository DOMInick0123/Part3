import pyglet
from pyglet.window import key
from Player import Player
#wwwwfrom PlayerTesting import Player


player = Player()
label = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=100, y=100,
                          anchor_x='center', anchor_y='center')
speed = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=20,
                          x=100, y=200,
                          anchor_x='center', anchor_y='center')


if __name__ == '__main__':
    window = pyglet.window.Window(width=1900, height=1000, caption="Car")
    pyglet.gl.glClearColor(0.5, 0.5, 0.5, 0.5)

    @window.event
    def on_draw():
        window.clear()
        player.draw()
        label.draw()
        speed.draw()


    def update(dt):
        player.update_player(dt)
        #player.update_player(0.001)
        label.text = str(int(player.steering))
        speed.text = str(int(player.velocity_angular_car))

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


    pyglet.clock.schedule_interval(update, 1 / 50.0)
    #pyglet.clock.schedule_interval(update, 1 / 2.0)
    pyglet.app.run()
