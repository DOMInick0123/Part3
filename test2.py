import math
from multiprocessing import Process, Queue, Lock
import numpy as np
from random import choice, random, gauss
from Car import Car
import time

# constants
physics_engine = 5
acc_pedal = 6
brk_pedal = 6
lateral = 6
fc_c = 6
avg_dis = 6
best_car = Car()
population_size = 1024
process_count = 4
car_grid = np.zeros((acc_pedal, brk_pedal, lateral, fc_c, avg_dis), dtype=Car)


def to_grid(car):
    acc = int(
        acc_pedal / 2 * (math.tanh(8 * (car.avg_acc / car.alive_counter * physics_engine - 0.8)) + 1))
    brk = int(
        brk_pedal / 2 * (math.tanh(8 * (car.avg_brk / car.alive_counter * physics_engine - 0.15)) + 1))
    lat = min(int(car.max_lateral / 20000 * lateral), lateral - 1)
    if car.fuel == 100.:
        fc = 0
    else:
        fc = min(int(math.tanh(car.distance / (100. - car.fuel) / 3000) * fc_c), fc_c - 1)
    mid = min(avg_dis - 1, max(0, int((math.tanh(car.mid/car.alive_counter*25) + 0.5)*avg_dis)))
    grid_car = car_grid[acc, brk, lat, fc, mid]
    if grid_car == 0 or car.score > grid_car.score:
        car_grid[acc, brk, lat, fc, mid] = car


def check_grid():
    try:
        networks = np.load('grid.npy', allow_pickle=True)
    except IOError:
        pass
    else:
        for network in networks:
            p = Car(network[0])
            p.alive_counter = network[1]
            p.avg_acc = network[2]
            p.avg_brk = network[3]
            p.max_lateral = network[4]
            p.distance = network[5]
            p.fuel = network[6]
            p.score = network[7]
            p.mid = network[8]
            to_grid(p)
    ap = np.zeros((acc_pedal,))
    bp = np.zeros((brk_pedal,))
    la = np.zeros((lateral,))
    fc = np.zeros((fc_c,))
    sp = np.zeros((avg_dis,))

    for i in range(acc_pedal):
        for j in range(brk_pedal):
            for l in range(lateral):
                for m in range(fc_c):
                    for k in range(avg_dis):
                        if not car_grid[i, j, l, m, k] == 0:
                            ap[i] += 1
                            bp[j] += 1
                            la[l] += 1
                            fc[m] += 1
                            sp[k] += 1
    print(ap)
    print(bp)
    print(la)
    print(fc)
    print(sp)


def check_best():
    treshold = 80
    better = 0
    try:
        networks = np.load('grid.npy', allow_pickle=True)
    except IOError:
        pass
    else:
        for network in networks:
            if network[7] > treshold:
                better += 1
    print(better)


if __name__ == '__main__':
    #check_grid()
    check_best()
