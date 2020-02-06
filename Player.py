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
rear_wheel_inertia = 20 * wheel_radius**2
total_inertia = rear_wheel_inertia + 2
car_inertia = 600.


def rpm_to_torque(rpm):
    x = rpm - 4500
    return -0.000017*x**2 - 0.034*x + 480


class Player(pyglet.sprite.Sprite):
    img = None

    def __init__(self):
        if self.img is None:
            self.img = pyglet.image.load('Car.png')
            self.img.anchor_x = 7
            self.img.anchor_y = 11
        pyglet.sprite.Sprite.__init__(self, self.img, x=800, y=500)
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
        car_mass = mass + self.fuel
        car_weight = car_mass * g
        weight_transfer = h_L * car_mass * self.acceleration_last
        weight_front = c_L * car_weight - weight_transfer
        weight_rear = b_L * car_weight + weight_transfer

        sin_rotation = math.sin(math.radians(self.rotation))
        cos_rotation = math.cos(math.radians(self.rotation))
        matrix = np.array(((cos_rotation, -sin_rotation), (sin_rotation, cos_rotation)))
        v_long, v_lat = np.matmul(matrix, self.velocity)
        steering_angle = math.radians(22*self.rot)
        if v_long == 0:
            alpha_front = 0
            alpha_rear = 0
        else:
            alpha_front = np.arctan((v_lat + self.velocity_angular_car*b_L)/v_long) - steering_angle*np.sign(v_long)
            alpha_rear = np.arctan((v_lat - self.velocity_angular_car*c_L)/v_long)
        force_lat_front = alpha_front * cornering_stiffness
        force_lat_rear = alpha_rear * cornering_stiffness
        if abs(alpha_front) > math.radians(3):
            force_lat_front /= 5000 / weight_front
        if abs(alpha_rear) > math.radians(3):
            force_lat_rear /= 5000 / weight_rear
        rpm = v_long * 30 * gear_ratios[self.gear] * differential_ratio / (math.pi + wheel_radius)
        self.change_gear(rpm)
        torque_drive = rpm_to_torque(rpm) * gear_ratios[self.gear] * differential_ratio * self.acceleration_pedal
        force_drive = torque_drive / wheel_radius
        self.fuel -= dt*force_drive/100000
        force_drag_long = drag * v_long * abs(v_long)
        force_drag_lat = drag * v_lat * abs(v_lat)
        force_rolling_resistance_long = rr * v_long
        force_rolling_resistance_lat = rr * v_lat
        force_resistance_long = force_rolling_resistance_long + force_drag_long
        force_resistance_lat = force_rolling_resistance_lat + force_drag_lat
        force_braking = self.braking_pedal * braking_constant
        sin_sigma = np.sin(steering_angle)
        cos_sigma = np.cos(steering_angle)
        force_long = force_drive + force_lat_front*sin_sigma - force_resistance_long - force_braking
        force_lat = force_lat_rear + force_lat_front*cos_sigma - force_resistance_lat
        torque_car = cos_sigma*force_lat_front*front_axle-force_lat_rear*rear_axle
        acceleration = np.array((force_long, force_lat))/car_mass
        angular_acceleration = torque_car/car_inertia
        matrix = np.array(((cos_rotation, sin_rotation), (-sin_rotation, cos_rotation)))
        a = np.matmul(matrix, acceleration)
        print(a)
        self.velocity += a
        self.x += 5.5*dt * self.velocity[0]
        self.y += 5.5*dt * self.velocity[1]
        self.velocity_angular_car += dt*angular_acceleration
        self.rotation += dt*math.degrees(self.velocity_angular_car)

    def change_gear(self, rpm):
        if rpm > 4600 and self.gear < 5:
            self.gear += 1
        elif rpm < 2000 and self.gear > 0:
            self.gear -= 1

