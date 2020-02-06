from random import random, gauss

import math
import pyglet
import numpy as np

torque = 500.
wheel_radius = 0.4
drag = 0.4
rr = 12.
braking_constant = 15000.
mass = 1000.
g = 9.81
weight = mass * g
cornering_stiffness = 1000.
length = 4.
wheelbase = 2.6
front_axle = 1.5
rear_axle = 1.1
centre_height = 0.6
h_L = centre_height / wheelbase
b_L = front_axle / wheelbase
c_L = rear_axle / wheelbase
gear_ratios = [2.66, 1.78, 1.3, 1., 0.74, 0.5]
differential_ratio = 4.
transmission_efficiency = 1.
tyre_friction = 1.2
traction_constant = 10.
rear_wheel_inertia = 20 * wheel_radius ** 2
total_inertia = rear_wheel_inertia + 2
car_inertia = 600.


def rpm_to_torque(rpm):
    x = rpm - 4500
    return -0.000017 * x ** 2 - 0.034 * x + 480


class Player(pyglet.sprite.Sprite):
    img = None

    def __init__(self):
        if self.img is None:
            self.img = pyglet.image.load('Car.png')
            self.img.anchor_x = 7
            self.img.anchor_y = 11
        pyglet.sprite.Sprite.__init__(self, self.img, x=10, y=100)
        self.rotation = 0
        self.rot = 0.
        self.acceleration_pedal = 0.
        self.braking_pedal = 0.
        self.gear = 0
        self.acceleration_last = 0
        self.velocity_angular_car = 0
        self.speed_long = 0
        self.speed_lat = 0

        self.velocity = np.zeros((2, 1))
        self.fuel = 100.

    def update_player(self, dt=0.02):

        steering_angle = math.radians(20 * self.rot)
        sin_delta = math.sin(steering_angle)
        cos_delta = math.cos(steering_angle)
        car_mass = mass + self.fuel
        car_weight = car_mass * g
        weight_transfer = h_L * car_mass * self.acceleration_last
        weight_front = c_L * car_weight - weight_transfer
        weight_rear = b_L * car_weight + weight_transfer
        force_front_max = weight_front * tyre_friction
        force_rear_max = weight_rear * tyre_friction
        velocity_angular_wheel = self.speed_long / wheel_radius
        rpm = 30 * gear_ratios[self.gear] * differential_ratio / math.pi * velocity_angular_wheel
        torque_drive = rpm_to_torque(rpm) * gear_ratios[self.gear] * differential_ratio * self.acceleration_pedal
        force_drive = torque_drive / wheel_radius
        self.fuel -= dt * force_drive / 100000
        force_drag_long = drag * self.speed_long * abs(self.speed_long)
        force_rolling_resistance_long = rr * self.speed_long
        force_braking = self.braking_pedal * braking_constant
        if self.speed_long < 0:
            force_braking *= -1
        if force_drive > force_rear_max:
            force_drive -= 0.5 * (force_drive - force_rear_max)
        if force_braking * 0.4 > force_rear_max:
            force_braking -= 0.25 * (0.4 * force_braking - force_rear_max)
        if force_braking * 0.6 > force_front_max:
            force_braking -= 0.25 * (0.6 * force_braking - force_front_max)
        omega = self.speed_long * sin_delta / wheelbase
        force_cornering = omega * cornering_stiffness
        force_long = force_drive - force_drag_long - force_rolling_resistance_long - force_braking - abs(
            force_cornering)
        force_lat_rear = 10 * force_cornering * b_L
        force_lat_front = 10 * force_cornering * c_L
        self.speed_lat *= 0.95
        force_lat = 0
        if abs(force_lat_rear) > force_rear_max:
            force_lat += 1.4 * force_lat_rear
            force_long -= abs(force_lat_rear / 5)
        if abs(force_lat_front) > force_front_max:
            force_lat += 1.4 * force_lat_front
        self.change_gear(rpm)
        acceleration_long = force_long / car_mass
        self.acceleration_last = acceleration_long
        acceleration_lat = force_lat / car_mass
        self.velocity_angular_car *= 0.95
        acceleration_angular = force_cornering / 800
        self.velocity_angular_car += dt * acceleration_angular
        self.rotation += dt * 180 * self.velocity_angular_car / math.pi
        sin_rotation = math.sin(math.radians(self.rotation))
        cos_rotation = math.cos(math.radians(self.rotation))
        matrix = np.array(((cos_rotation, sin_rotation), (-sin_rotation, cos_rotation)))
        self.speed_long += dt * acceleration_long
        self.speed_lat += dt * acceleration_lat
        vel_x, vel_y = matrix.dot(np.array([self.speed_long, self.speed_lat]))
        self.x += 5.5 * dt * vel_x
        self.y += 5.5 * dt * vel_y

    def change_gear(self, rpm):
        if rpm > 4600 and self.gear < 5:
            self.gear += 1
        elif rpm < 2000 and self.gear > 0:
            self.gear -= 1

