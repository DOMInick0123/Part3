from random import random, gauss

import math
import pyglet
import numpy as np
wheel_radius = 0.4
drag = 0.4
rr = 12.
braking_constant = 13000.
mass = 1200.
g = 9.81
weight = mass * g
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
final_ratios = [differential_ratio * i for i in gear_ratios]
transmission_efficiency = 1.
rear_wheel_inertia = 10 * wheel_radius**2
car_inertia = 600.
friction_static = 0.8
friction_kinetic = 0.6
to_rad = 30 / (math.pi * wheel_radius)


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
        pyglet.sprite.Sprite.__init__(self, self.img, x=100, y=500)
        self.rotation = 0
        self.steering = 0.
        self.acceleration_pedal = 0.
        self.braking_pedal = 0.
        self.gear = 0
        self.acceleration_last = 0
        self.velocity_angular_car = 0

        self.velocity = np.array((0, 0)).reshape((2, 1))
        self.fuel = 100.

    def update_player(self, dt=0.02):
        car_mass = mass + self.fuel
        car_weight = car_mass * g
        sin_rotation = math.sin(math.radians(self.rotation))
        cos_rotation = math.cos(math.radians(self.rotation))
        matrix = np.array(((cos_rotation, -sin_rotation), (sin_rotation, cos_rotation)))
        v = np.matmul(matrix, self.velocity).reshape((2,))
        speed = sum(v * v) ** 0.5
        #Weight on wheels
        weight_transfer = h_L * car_mass * self.acceleration_last
        weight_front = c_L * car_weight - weight_transfer
        weight_rear = b_L * car_weight + weight_transfer
        force_front_max = weight_front * friction_static
        force_rear_max = weight_rear * friction_static

        rpm = abs(v[0]) * final_ratios[self.gear] * to_rad
        force_drive = rpm_to_torque(rpm) * final_ratios[self.gear] * self.acceleration_pedal / wheel_radius
        force_brake = -self.braking_pedal * braking_constant

        self.fuel -= dt * force_drive / 100000
        self.change_gear(rpm)

        force_drag = -drag * v * speed
        force_rolling_resistance = -rr * v
        if force_drive > force_rear_max:
            force_drive -= 0.5 * (force_drive - force_rear_max)
        if force_brake * 0.4 > force_rear_max:
            force_brake -= 0.25 * (0.4 * force_brake - force_rear_max)
        if force_brake * 0.6 > force_front_max:
            force_brake -= 0.25 * (0.6 * force_brake - force_front_max)
        force_long = force_drive + force_brake
        if v[0] > 0:
            steering_angle = math.radians(22 * self.steering)
            if steering_angle != 0:
                sin_delta = math.sin(steering_angle)
                omega = v[0] * sin_delta / wheelbase
                alpha_front = math.atan((v[1] + omega * front_axle) / v[0]) - steering_angle
                alpha_rear = math.atan((v[1] - omega * rear_axle) / v[0])
                print(alpha_front)
            else:
                alpha_front = math.atan(v[1] / v[0])
                alpha_rear = alpha_front
            if abs(alpha_front) > 0.03:
                force_lateral_front = 1.2 * weight_front * np.sign(alpha_front)
            else:
                force_lateral_front = 40 * weight_front * alpha_front
            if abs(alpha_rear) > 0.03:
                force_lateral_rear = 1.2 * weight_rear * np.sign(alpha_rear)
            else:
                force_lateral_rear = 40 * weight_rear * alpha_rear
            force_front = math.cos(steering_angle) * force_lateral_front
            #force_cornering = force_lateral_rear + force_front
            force_cornering = 0
            #print(force_cornering)
            torque_yaw = force_front * front_axle - force_lateral_rear * rear_axle
            angular_acceleration = torque_yaw/car_inertia
            self.velocity_angular_car = angular_acceleration
        else:
            force_cornering = 0
        force_final = np.array((force_long, force_cornering)) + force_drag + force_rolling_resistance
        acceleration = force_final/car_mass
        self.acceleration_last = acceleration[0]
        v += dt*acceleration
        if self.velocity_angular_car != 0:
            self.rotation += dt * self.velocity_angular_car
            sin_rotation = math.sin(math.radians(self.rotation))
            cos_rotation = math.cos(math.radians(self.rotation))
        matrix = np.array(((cos_rotation, sin_rotation), (-sin_rotation, cos_rotation)))
        self.velocity = np.matmul(matrix, v.reshape((2, 1)))
        self.x += 5.5*dt * self.velocity[0]
        self.y += 5.5*dt * self.velocity[1]
        #self.velocity_angular_car += dt*angular_acceleration


    def change_gear(self, rpm):
        if rpm > 4600 and self.gear < 5:
            self.gear += 1
        elif rpm < 2000 and self.gear > 0:
            self.gear -= 1

