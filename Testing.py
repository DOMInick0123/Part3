import math
from multiprocessing import Process, Queue, Lock
import numpy as np
from random import choice, random, gauss
import time
import matplotlib.pyplot as plt

# constants
from PIL import Image

b = 1.2  # in m, distance from CG to front axle
c = 1.8  # in m, distance from CG to rear axle
wheelbase = b + c  # in m, distance between axles
h = 0.25  # in m, height of CG from ground
front_wing = 1.1  # in m, distance from front axle to front of the car
rear_wing = 0.7  # in m, distance from rear axle to rear of the car
mass = 750.  # in kg
length = wheelbase + front_wing + rear_wing  # in m
width = 1.8  # in m
inertia = mass*(h**2+length**2)/12  # in kg.m**2
wheel_radius = 0.4  # in m
drag = 0.4  # constant for drag resistance
rr = 30 * drag  # constant for rolling resistance
ca_r = -5.2  # cornering stiffness for rear wheels
ca_f = -5.  # cornering stiffness for front wheels
gear_ratios = [2.66, 1.78, 1.3, 1., 0.74, 0.5]
differential_ratio = 3.42
g = 9.81
greyscale = 100
scale = 2

# precalculated values
h_L = h / wheelbase
b_L = b / wheelbase
c_L = c / wheelbase
front_cg = b+front_wing * scale
side_cg = width/2 * scale
final_ratios = [differential_ratio * i for i in gear_ratios]
a_rpm = -17/1015000
b_rpm = 2363/20300
c_rpm = 8596/29

transmission_efficiency = 0.8
rad_to_rpm = 30 / (math.pi * wheel_radius)


def rpm_to_torque(rpm):
    return max(((a_rpm * rpm + b_rpm) * rpm + c_rpm), 1000)


def thread_function(in_q, out_q):
    while 1:
        car, b = in_q.get()
        if car is None:
            break
        while car.alive:
            car.think()
            for j in range(5):
                car.update_car(0.01)
        out_q.put((car.score, b))


class Car:

    def __init__(self, network, x, y, rot, track, checkpoints, fuel=100., braking_constant=20000, tyre_grip=2.5):
        self.track = track
        self.pos_x = x
        self.pos_y = y
        self.rot_rad = math.radians(rot)
        self.sin_rotation = math.sin(self.rot_rad)
        self.cos_rotation = math.cos(self.rot_rad)
        self.steering = 0.
        self.acceleration_pedal = 0.
        self.braking_pedal = 0.
        self.gear = 0
        self.rpm = 1000.
        self.acceleration_last = 0
        self.velocity_x = 0.
        self.velocity_y = 0.
        self.velocity_local_x = 0.
        self.velocity_local_y = 0.
        self.velocity_angular = 0.
        self.fuel = fuel
        self.score = 0.
        self.distance = 0.
        self.checkpoint = 0
        self.dis_last_check = 0.
        self.alive_counter = 0
        self.alive = True
        self.network = network
        self.checkpoints = checkpoints
        self.braking_constant = braking_constant
        self.tyre_grip = tyre_grip

    def update_car(self, dt=0.01):
        if self.alive:
            self.alive_counter += 1
        else:
            return

        # collision detection
        fx = self.cos_rotation * front_cg
        fy = self.sin_rotation * front_cg
        front_x = self.pos_x + fx
        front_y = self.pos_y - fy
        back_x = self.pos_x - fx
        back_y = self.pos_y + fy
        side_x = self.sin_rotation * side_cg
        side_y = self.cos_rotation * side_cg
        left_front_x = front_x + side_x
        left_front_y = front_y + side_y
        right_front_x = front_x - side_x
        right_front_y = front_y - side_y
        left_rear_x = back_x + side_x
        left_rear_y = back_y + side_y
        right_rear_x = back_x - side_x
        right_rear_y = back_y - side_y
        if self.track[int(left_front_y)][int(left_front_x)] > greyscale or self.track[int(right_front_y)][int(right_front_x)] > greyscale or \
                self.track[int(left_rear_y)][int(left_rear_x)] > greyscale or self.track[int(right_rear_y)][int(right_rear_x)] > greyscale:
            self.score += (self.distance-self.dis_last_check)*0.01
            self.score *= 0.7
            self.alive = False
            return
        check_x, check_y = self.checkpoints[self.checkpoint]
        if (self.pos_x - check_x) ** 2 + (self.pos_y - check_y) ** 2 < 175:
            self.checkpoint += 1
            self.score = 5 * self.checkpoint - 0.001 * self.alive_counter
            self.dis_last_check = self.distance

        # transforming velocity from global to local coordinates
        self.velocity_local_x = self.cos_rotation * self.velocity_x - self.sin_rotation * self.velocity_y
        self.velocity_local_y = self.cos_rotation * self.velocity_y + self.sin_rotation * self.velocity_x
        steering_angle = 0.38 * self.steering  # angle in range -0.38 to 0.38 rad

        # Weight on wheels
        car_mass = mass + self.fuel
        car_weight = car_mass * g
        weight_transfer = h_L * car_mass * self.acceleration_last
        weight_front = c_L * car_weight - weight_transfer
        weight_rear = b_L * car_weight + weight_transfer

        # Steering forces
        yaw_speed_front = b * self.velocity_angular
        yaw_speed_rear = c * self.velocity_angular
        alpha_front = math.atan2(self.velocity_local_y + yaw_speed_front, abs(self.velocity_local_x)) - np.sign(
            self.velocity_local_x) * steering_angle
        alpha_rear = math.atan2(self.velocity_local_y - yaw_speed_rear, abs(self.velocity_local_x))
        tg = self.tyre_grip + self.velocity_local_x / 8.  # downforce
        force_lateral_front = min(max(ca_f * alpha_front, -tg), tg) * weight_front
        force_lateral_rear = min(max(ca_f * alpha_rear, -tg), tg) * weight_rear
        force_lat = force_lateral_front * math.cos(steering_angle) + force_lateral_rear

        # traction force from engine and brakes
        self.rpm = self.velocity_local_x * final_ratios[self.gear] * rad_to_rpm
        torque_engine = rpm_to_torque(self.rpm) * self.acceleration_pedal
        force_drive = torque_engine * final_ratios[self.gear] * transmission_efficiency / wheel_radius
        force_brake = self.braking_constant * self.braking_pedal
        self.fuel -= dt * torque_engine / 100000
        force_traction = force_drive - force_brake * np.sign(self.velocity_local_x)
        force_resistance = (drag * self.velocity_local_x + rr) * self.velocity_local_x
        force_long = force_traction - force_resistance

        acceleration_local_x = force_long / car_mass
        acceleration_local_y = force_lat / car_mass
        self.acceleration_last = acceleration_local_x
        acceleration_x = self.cos_rotation * acceleration_local_x + self.sin_rotation * acceleration_local_y
        acceleration_y = -self.sin_rotation * acceleration_local_x + self.cos_rotation * acceleration_local_y
        self.velocity_x += dt * acceleration_x
        self.velocity_y += dt * acceleration_y

        angular_torque = force_lateral_front * b - force_lateral_rear * c

        speed = (self.velocity_x ** 2 + self.velocity_y ** 2) ** 0.5
        self.distance += dt*speed
        if self.alive_counter >= 140000 or (speed < 8 and self.alive_counter > 300) or (speed < 0.5 and self.acceleration_pedal == 0) or self.checkpoint == len(self.checkpoints):
            self.score += (self.distance - self.dis_last_check) * 0.01
            self.alive = False
            return

        acceleration_angular = angular_torque / inertia
        self.velocity_angular += dt * acceleration_angular

        if self.velocity_angular != 0:
            self.rot_rad += dt * self.velocity_angular
            self.sin_rotation = math.sin(self.rot_rad)
            self.cos_rotation = math.cos(self.rot_rad)

        self.pos_x += scale * dt * self.velocity_x
        self.pos_y += scale * dt * self.velocity_y

    def think(self):
        cos_30 = math.cos(self.rot_rad + math.radians(30))
        sin_30 = math.sin(self.rot_rad + math.radians(30))
        cos_60 = math.cos(self.rot_rad + math.radians(60))
        sin_60 = math.sin(self.rot_rad + math.radians(60))
        left = self.sensor(-cos_30, sin_30, 30)
        right = self.sensor(sin_60, cos_60, 30)
        inputs = [self.velocity_local_x / 65.234, self.velocity_local_y / 30., self.velocity_angular, left,
                  self.sensor(-cos_60, sin_60, 120),
                  self.sensor(self.sin_rotation, self.cos_rotation, 200), self.sensor(sin_30, cos_30, 120), right]
        outputs = []
        for layer in self.network:
            outputs.clear()
            for i in range(len(layer[1])):
                val = layer[1][i]
                for j in range(len(layer[0][i])):
                    val += layer[0][i][j] * inputs[j]
                outputs.append(math.tanh(val))
            inputs = outputs.copy()
        self.acceleration_pedal = (outputs[0] + 1) / 2
        self.braking_pedal = (outputs[1] + 1) / 2
        self.steering = outputs[2]
        if self.rpm > 5500:
            self.gear += 1
            self.gear = min(5, self.gear)
        if self.rpm < 2000:
            self.gear -= 1
            self.gear = max(0, self.gear)

    def sensor(self, sin, cos, ran):
        for i in np.arange(0, ran, 0.5):
            if self.track[int(self.pos_y - i * sin*scale)][int(self.pos_x + i * cos*scale)] > greyscale:
                return i / ran
        return 1


def graphs():
    try:
        run1 = np.load('run1/progress.npy', allow_pickle=True).T
        run2 = np.load('run2/progress.npy', allow_pickle=True).T
        run3 = np.load('run3/progress.npy', allow_pickle=True).T
        run4 = np.load('run4/progress.npy', allow_pickle=True).T
    except IOError:
        pass
    else:
        scores1 = run1[0]/1.10
        filled1 = run1[1]/77.76
        avg_score1 = run1[2]/1.10
        scores2 = run2[0] / 1.10
        filled2 = run2[1] / 77.76
        avg_score2 = run2[2] / 1.10
        scores3 = run3[0] / 1.10
        filled3 = run3[1] / 77.76
        avg_score3 = run3[2] / 1.10
        scores4 = run4[0] / 1.10
        filled4 = run4[1] / 77.76
        avg_score4 = run4[2] / 1.10
        plt.plot(range(len(scores1)), scores1, label='run1')
        plt.plot(range(len(scores2)), scores2, label='run2')
        plt.plot(range(len(scores3)), scores3, label='run3')
        plt.plot(range(len(scores4)), scores4, label='run4')
        plt.ylabel('Best Fitness in %')
        plt.xlabel('Generation (-)')
        plt.title('Fitness of the best individual each generation')
        plt.legend()
        plt.show()
        plt.plot(range(len(filled1)), filled1, label='run1')
        plt.plot(range(len(filled2)), filled2, label='run2')
        plt.plot(range(len(filled3)), filled3, label='run3')
        plt.plot(range(len(filled4)), filled4, label='run4')
        plt.ylabel('Percentage filled %')
        plt.xlabel('Generation (-)')
        plt.title('Percentage of the grid filled each generation')
        plt.legend()
        plt.show()
        plt.plot(range(len(avg_score1)), avg_score1, label='run1')
        plt.plot(range(len(avg_score2)), avg_score2, label='run2')
        plt.plot(range(len(avg_score3)), avg_score3, label='run3')
        plt.plot(range(len(avg_score4)), avg_score4, label='run4')
        plt.ylabel('Average Fitness in %')
        plt.xlabel('Generation (-)')
        plt.title('Average fitness of all individuals in the grid each generation')
        plt.legend()
        plt.show()


def pick_individuals():
    top = []
    current = np.load('run1/grid.npy', allow_pickle=True)
    cr = [car[0] for car in current if car[7] > 20]
    np.random.shuffle(cr)
    for i in range(500):
        top.append(cr[i])
    current = np.load('run2/grid.npy', allow_pickle=True)
    cr = [car[0] for car in current if car[7] > 20]
    np.random.shuffle(cr)
    for i in range(500):
        top.append(cr[i])
    current = np.load('run3/grid.npy', allow_pickle=True)
    cr = [car[0] for car in current if car[7] > 20]
    np.random.shuffle(cr)
    for i in range(500):
        top.append(cr[i])
    current = np.load('run4/grid.npy', allow_pickle=True)
    cr = [car[0] for car in current if car[7] > 20]
    np.random.shuffle(cr)
    for i in range(500):
        top.append(cr[i])
    np.save('individuals.npy', top, allow_pickle=True)


def check_tracks():
    top = np.load('individuals.npy', allow_pickle=True)
    in_q = Queue()
    out_q = Queue()
    processes = []
    for _ in range(16):
        x = Process(target=thread_function, args=(in_q, out_q))
        x.start()
        processes.append(x)
    # cars = [0]*2000
    # track = np.array(Image.open('monza.jpg').transpose(Image.FLIP_TOP_BOTTOM))
    # checkpoints = (
    #     (2317, 488), (2257, 514), (1638, 339), (833, 1357), (718, 1582), (655, 1628), (453, 2329), (988, 2434),
    #     (2654, 1172), (2881, 1141), (3014, 1109), (4978, 1320), (4158, 732))
    # for i in range(4):
    #     for j in range(500):
    #         in_q.put((Car(top[i*500+j], 4158., 732., 170, track, checkpoints), i*500+j))
    #     for j in range(500):
    #         a, b = out_q.get()
    #         if j % 25 == 0:
    #             print(i*500+j)
    #         cars[b] = a
    # cars = np.asarray(cars)/(len(checkpoints)*5)
    # np.save('score1.npy', cars, allow_pickle=True)
    # cars = [0]*2000
    # track = np.array(Image.open('yasmarina.jpg').transpose(Image.FLIP_TOP_BOTTOM))
    # checkpoints = (
    #     (1571, 1439), (1789, 1909), (1941, 2306), (2293, 2561), (2449, 2805), (2544, 2892), (2667, 2991), (2581, 3005), (404, 2096),
    #     (394, 1971), (229, 1130), (470, 250), (559, 242), (630, 248), (762, 480), (379, 960), (441, 1208),
    #     (601, 1231), (794, 1219), (708, 1539), (567, 1939), (1031, 1818))
    # for i in range(4):
    #     for j in range(500):
    #         in_q.put((Car(top[i * 500 + j], 1031., 1818., 45, track, checkpoints), i * 500 + j))
    #     for j in range(500):
    #         a, b = out_q.get()
    #         if j % 25 == 0:
    #             print(i * 500 + j)
    #         cars[b] = a
    # cars = np.asarray(cars)/(len(checkpoints)*5)
    # np.save('score2.npy', cars, allow_pickle=True)
    checkpoints = (
        (987, 277), (810, 326), (662, 291), (502, 290), (287, 418), (324, 604), (357, 704), (579, 1043), (723, 1093),
        (1738, 869), (1822, 825), (1995, 850), (2181, 846), (2266, 684), (2157, 459), (1980, 364), (1865, 303),
        (1818, 342), (1763, 369), (1435, 319), (1218, 325), (1165, 356))
    cars = [0]*2000
    track = np.array(Image.open('testtrack.jpg').transpose(Image.FLIP_TOP_BOTTOM))
    # for i in range(4):
    #     for j in range(500):
    #         in_q.put((Car(top[i * 500 + j], 1100., 320., 160, track, checkpoints, fuel=900), i * 500 + j))
    #     for j in range(500):
    #         a, b = out_q.get()
    #         if j % 25 == 0:
    #             print(i * 500 + j)
    #         cars[b] = a
    # cars = np.asarray(cars)/(len(checkpoints)*5)
    # np.save('score3.npy', cars, allow_pickle=True)
    # cars = [0]*2000
    # for i in range(4):
    #     for j in range(500):
    #         in_q.put((Car(top[i * 500 + j], 1100., 320., 160, track, checkpoints, braking_constant=8000), i * 500 + j))
    #     for j in range(500):
    #         a, b = out_q.get()
    #         if j % 25 == 0:
    #             print(i * 500 + j)
    #         cars[b] = a
    # cars = np.asarray(cars)/(len(checkpoints)*5)
    # np.save('score4.npy', cars, allow_pickle=True)
    for i in range(4):
        for j in range(500):
            in_q.put((Car(top[i * 500 + j], 1100., 320., 160, track, checkpoints, tyre_grip=0.1), i * 500 + j))
        for j in range(500):
            a, b = out_q.get()
            if j % 25 == 0:
                print(i * 500 + j)
            cars[b] = a
    cars = np.asarray(cars)/(len(checkpoints)*5)
    np.save('score5.npy', cars, allow_pickle=True)
    for _ in range(16):
        in_q.put((None, None))


def sum_scores():
    s1 = np.load('score1.npy', allow_pickle=True)*100
    s2 = np.load('score2.npy', allow_pickle=True)*100
    s3 = np.load('score3.npy', allow_pickle=True)*100
    s4 = np.load('score4.npy', allow_pickle=True)*100
    s5 = np.load('score5.npy', allow_pickle=True)*100
    s = (s1 + s2 + s3 + s4 + s5)/5
    best = (np.argmax(s1), np.argmax(s2), np.argmax(s3), np.argmax(s4), np.argmax(s5), np.argmax(s))
    test1 = []
    test2 = []
    test3 = []
    test4 = []
    test5 = []
    summ = []
    width = 0.13
    for br in best:
        test1.append(s1[br])
        test2.append(s2[br])
        test3.append(s3[br])
        test4.append(s4[br])
        test5.append(s5[br])
        summ.append(s[br])
    plt.bar(np.arange(6)-width*2.5, test1, width, label='Test1')
    plt.bar(np.arange(6)-width*1.5, test2, width, label='Test2')
    plt.bar(np.arange(6)-width*0.5, test3, width, label='Test3')
    plt.bar(np.arange(6)+width*0.5, test4, width, label='Test4')
    plt.bar(np.arange(6)+width*1.5, test5, width, label='Test5')
    plt.bar(np.arange(6)+width*2.5+0.025, summ, width+0.05, label='Average', color='k')
    plt.xticks(np.arange(6), labels=("Best in test 1", "Best in test 2", "Best in test 3", "Best in test 4", "Best in test 5", "Best on average"))
    plt.xlabel("Individual")
    plt.ylabel("Score in %")
    plt.title("Scores of the best individual in each test")
    # plt.title("Sorted scores of individuals in each test")
    # plt.plot(range(2000), np.sort(s1), label='Test 1')
    # plt.plot(range(2000), np.sort(s2), label='Test 2')
    # plt.plot(range(2000), np.sort(s3), label='Test 3')
    # plt.plot(range(2000), np.sort(s4), label='Test 4')
    # plt.plot(range(2000), np.sort(s5), label='Test 5')
    # plt.plot(range(2000), np.sort(s), label='Average', color='k', linewidth=3)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    #check_grid()
    #pick_individuals()
    #graphs()
    #check_tracks()
    sum_scores()
