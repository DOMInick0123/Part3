from random import random, gauss
import math
import numpy as np
from PIL import Image


# constants
b = 1.5  # in m, distance from CG to front axle
c = 1.1  # in m, distance from CG to rear axle
wheelbase = b + c  # in m, distance between axles
h = 0.6  # in m, height of CG from ground
front_wing = 0.9  # in m, distance from front axle to front of the car
rear_wing = 0.7  # in m, distance from rear axle to rear of the car
mass = 1200.  # in kg
inertia = 1200.  # in kg.m
length = wheelbase + front_wing + rear_wing  # in m
width = 1.8  # in m
wheel_radius = 0.4  # in m
drag = 0.4  # constant for drag resistance
rr = 30 * drag  # constant for rolling resistance
ca_r = -21.2  # cornering stiffness for rear wheels
ca_f = -21  # cornering stiffness for front wheels
tyre_grip = 1.2  # diameter of friction circle
braking_constant = 13000.
cornering_stiffness = 1000.
gear_ratios = [2.66, 1.78, 1.3, 1., 0.74, 0.5]
differential_ratio = 4.
g = 9.81

# precalculated values
h_L = h / wheelbase
b_L = b / wheelbase
c_L = c / wheelbase
front_cg = b + front_wing
side_cg = width / 2
final_ratios = [differential_ratio * i for i in gear_ratios]

transmission_efficiency = 1.
to_rad = 30 / (math.pi * wheel_radius)

track = np.array(Image.open('track.jpeg').transpose(Image.FLIP_TOP_BOTTOM))
#3.889186897169014

def rpm_to_torque(rpm):
    x = rpm - 4500
    return -0.000017 * x ** 2 - 0.034 * x + 480


def sigmoid(val):
    if val < -700:
        return 0
    return 1/(1+math.e**(-val))


class Player:
    def __init__(self, network=None):
        self.x = 10.
        self.y = 120.
        self.rotation = math.radians(-52)
        self.sin_rotation = math.sin(self.rotation)
        self.cos_rotation = math.cos(self.rotation)
        self.steering = 0.
        self.acceleration_pedal = 0.
        self.braking_pedal = 0.
        self.gear = 0
        self.acceleration_last = 0

        self.velocity = np.zeros((2,))
        self.velocity_angular = 0.
        self.fuel = 100.
        self.score = 0.
        self.distance = 0.
        self.alive_counter = 0
        self.alive = True
        self.death_counter = 0
        self.avg_acc = 0.
        self.avg_brk = 0.
        self.avg_fc = 0.
        if network is None:
            self.network = []
            layers = (4, 3)
            for i in range(len(layers) - 1):
                weights = []
                bias = []
                for j in range(layers[i + 1]):
                    row = []
                    for k in range(layers[i]):
                        row.append(random() * 2 - 1)
                    bias.append(random() * 50 - 25)
                    weights.append(row)
                self.network.append((weights, bias))
        else:
            self.network = network

    def update_player(self, dt=0.01):
        self.alive_counter += 1
        # collision detection
        front_x = self.x + self.cos_rotation * front_cg
        front_y = self.y - self.sin_rotation * front_cg
        left_x = front_x + self.sin_rotation * side_cg
        left_y = front_y + self.cos_rotation * side_cg
        right_x = front_x - self.sin_rotation * side_cg
        right_y = front_y - self.cos_rotation * side_cg
        if track[int(left_y)][int(left_x)] > 20 or track[int(right_y)][int(right_x)] > 20:
            self.score = self.distance * 0.01 - 0.001 * self.alive_counter
            self.alive = False
            return

        # transforming velocity from global to local coordinates
        matrix = np.array(((self.cos_rotation, -self.sin_rotation), (self.sin_rotation, self.cos_rotation)))
        v = np.matmul(matrix, self.velocity).reshape((2,))
        steering_angle = 0.38 * self.steering  # angle in range -0.38 to 0.38 rad

        # Weight on wheels
        car_mass = mass + self.fuel
        car_weight = car_mass * g
        weight_transfer = h_L * car_mass * self.acceleration_last
        weight_front = c_L * car_weight - weight_transfer
        weight_rear = b_L * car_weight + weight_transfer
        force_front_max = weight_front * tyre_grip
        force_rear_max = weight_rear * tyre_grip

        # traction force from engine and brakes
        rpm = v[0] * final_ratios[self.gear] * to_rad
        force_drive = rpm_to_torque(rpm) * final_ratios[self.gear] * self.acceleration_pedal / wheel_radius
        if force_drive > weight_rear:
            force_drive -= 0.5 * (force_drive - weight_rear)
        if self.braking_pedal > 0:
            if v[0] < 0.01:
                force_brake = 0
                v[0] = 0
            else:
                force_brake = self.braking_pedal * braking_constant
                if force_brake * 0.4 > force_front_max:
                    force_brake -= 0.25 * (0.4 * force_brake - force_front_max)
                if force_brake * 0.6 > force_rear_max:
                    force_brake -= 0.25 * (0.6 * force_brake - force_rear_max)
        else:
            force_brake = 0
        force_traction = force_drive - force_brake
        force_resistance = (drag * v + rr) * v
        force_long = force_traction - force_resistance[0]

        # Steering forces
        if v[0] > 15:
            cos_delta = math.cos(steering_angle)
            sideslip = math.atan2(v[1], v[0])
            alpha_front = sideslip + math.atan2(self.velocity_angular * b, v[0]) - steering_angle
            alpha_rear = sideslip - math.atan2(self.velocity_angular * c, v[0])
            force_lateral_front = ca_f * alpha_front
            force_lateral_rear = ca_r * alpha_rear
            force_lateral_front *= weight_front
            force_lateral_rear *= weight_rear
            torque = b * cos_delta * force_lateral_front - c * force_lateral_rear
            acceleration_angular = torque / inertia
            force_lat = force_lateral_front * cos_delta + force_lateral_rear - force_resistance[1]
            self.death_counter = 0
        else:
            acceleration_angular = v[0]*math.sin(steering_angle)/wheelbase
            force_lat = 0
            self.death_counter += 1
        if self.death_counter > 100 or self.alive_counter == 100000:
            self.score = self.distance*0.01-0.001*self.alive_counter
            self.alive = False
            return
        self.velocity_angular += dt * acceleration_angular

        self.fuel -= dt * force_drive / 100000
        self.change_gear(rpm)

        force_total = np.array((force_long, force_lat))

        acceleration = force_total / car_mass
        self.acceleration_last = acceleration[0]
        v += dt * acceleration
        self.distance += v[0]

        if self.velocity_angular != 0:
            self.rotation += dt * self.velocity_angular
            self.sin_rotation = math.sin(self.rotation)
            self.cos_rotation = math.cos(self.rotation)
        matrix = np.array(((self.cos_rotation, self.sin_rotation), (-self.sin_rotation, self.cos_rotation)))
        self.velocity = np.matmul(matrix, v.reshape((2, 1))).reshape((2,))
        self.x += dt * self.velocity[0]
        self.y += dt * self.velocity[1]

    def think(self):
        inputs = [self.velocity[0], self.sensor_front(), self.sensor_left(), self.sensor_right()]
        outputs = []
        for layer in self.network:
            outputs.clear()
            for i in range(len(layer[1])):
                val = layer[1][i]
                for j in range(len(layer[0][i])):
                    val += layer[0][i][j] * inputs[j]
                outputs.append(sigmoid(val))
            inputs = outputs.copy()
        self.acceleration_pedal = outputs[0]
        self.avg_acc += outputs[0]
        self.braking_pedal = outputs[1]
        self.avg_brk += outputs[1]
        self.steering = outputs[2]*2-1

    def change_gear(self, rpm):
        if rpm > 4600 and self.gear < 5:
            self.gear += 1
        elif rpm < 2000 and self.gear > 0:
            self.gear -= 1

    def sensor_front(self):
        for i in range(250):
            if track[int(self.y - i * self.sin_rotation)][int(self.x + i * self.cos_rotation)] > 20:
                return i
        return 500

    def sensor_left(self):
        for i in range(250):
            if track[int(self.y + i * self.cos_rotation)][int(self.x + i * self.sin_rotation)] > 20:
                return i
        return 500

    def sensor_right(self):
        for i in range(250):
            if track[int(self.y - i * self.cos_rotation)][int(self.x - i * self.sin_rotation)] > 20:
                return i
        return 500

    def copy(self):
        from copy import deepcopy
        return Player(deepcopy(self.network))

    def crossover(self, parent2):
        network = []
        for i in range(len(self.network)):
            weights = []
            bias = []
            for j in range(len(self.network[i][0])):
                row = []
                for k in range(len(self.network[i][0][j])):
                    if random() < 0.5:
                        row.append(self.network[i][0][j][k])
                    else:
                        row.append(parent2.network[i][0][j][k])
                weights.append(row)
                if random() < 0.5:
                    bias.append(self.network[i][1][j])
                else:
                    bias.append(parent2.network[i][1][j])
            network.append((weights, bias))
        p = Player(network)
        p.mutate()
        return p

    def mutate(self):
        for layer in self.network:
            for i in range(len(layer[0])):
                for j in range(len(layer[0][i])):
                    rand = random()
                    if rand < 0.5:
                        layer[0][i][j] += gauss(0, 1) / 50
                    elif rand > 0.9:
                        layer[0][i][j] = random() * 2 - 1
            for i in range(len(layer[1])):
                rand = random()
                if rand < 0.5:
                    layer[1][i] += gauss(0, 1) / 2
                elif rand > 0.9:
                    layer[1][i] = random() * 50 - 25
