import math
from multiprocessing import Process, Queue, Lock
import numpy as np
from random import choice, random, gauss
from Car import Car
import time

# constants
physics_engine = 5
acc_pedal = 16
brk_pedal = 8
avg_sp = 8
lateral = 6
fc_c = 16
best_car = Car()
population_size = 800
process_count = 3
car_grid = np.zeros((acc_pedal, brk_pedal, avg_sp, lateral, fc_c), dtype=Car)


def mutate(grid):
    if len(grid) == 0:
        return Car()
    c = choice(grid)
    while c[7] > best_car.score/10:
        c = choice(grid)
    from copy import deepcopy
    car1 = deepcopy(c[0])
    car2 = deepcopy(choice(grid)[0])
    nn = []
    for i in range(len(car1)):
        weights = []
        bias = []
        for j in range(len(car1[i][0])):
            row = []
            for k in range(len(car1[i][0][j])):
                if random() < 0.9:
                    row.append(car1[i][0][j][k])
                else:
                    row.append(car2[i][0][j][k])
            weights.append(row)
            if random() < 0.9:
                bias.append(car1[i][1][j])
            else:
                bias.append(car2[i][1][j])
        nn.append((weights, bias))
    for layer in nn:
        for i in range(len(layer[0])):
            for j in range(len(layer[0][i])):
                rand = random()
                if rand < 0.1:
                    layer[0][i][j] += gauss(0, 1) / 50
                    layer[0][i][j] = min(layer[0][i][j], 1)
                    layer[0][i][j] = max(layer[0][i][j], -1)
                elif rand > 0.99:
                    layer[0][i][j] = random() * 2 - 1
        for i in range(len(layer[1])):
            rand = random()
            if rand < 0.1:
                layer[1][i] += gauss(0, 1) / 50
                layer[1][i] = min(layer[1][i], 1)
                layer[1][i] = max(layer[1][i], -1)
            elif rand > 0.99:
                layer[1][i] = random() * 2 - 1
    return Car(nn)


def to_grid(car):
    global best_car, improv
    acc = int(
        acc_pedal / 2 * (math.tanh(8 * (car.avg_acc / car.alive_counter * physics_engine - 0.8)) + 1))
    brk = int(
        brk_pedal / 2 * (math.tanh(8 * (car.avg_brk / car.alive_counter * physics_engine - 0.15)) + 1))
    speed = min(int(math.tanh(car.distance/car.alive_counter*2) * avg_sp), lateral - 1)
    lat = min(int(car.max_lateral / 20000 * lateral), lateral - 1)
    if car.fuel == 100.:
        fc = 0
    else:
        fc = min(int(math.tanh(car.distance / (100. - car.fuel) / 1000) * fc_c), fc_c - 1)
    grid_car = car_grid[acc, brk, speed, lat, fc]
    if grid_car == 0 or car.score > grid_car.score:
        car_grid[acc, brk, speed, lat, fc] = car
        if car.score > best_car.score:
            best_car = car
        improv += 1


def thread_function(in_q, out_q):
    while 1:
        car = in_q.get()
        if car is None:
            break
        while car.alive:
            car.think()
            for j in range(physics_engine):
                car.update_car(0.01)
        out_q.put(car)


if __name__ == '__main__':
    gen = 1
    improv = 0
    cars = []
    networks = []
    in_q = Queue()
    out_q = Queue()
    processes = []
    for _ in range(process_count):
        x = Process(target=thread_function, args=(in_q, out_q))
        x.start()
        processes.append(x)
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
            to_grid(p)
    while 1:
        car_counter = 0
        improv = 0
        for _ in range(population_size):
            in_q.put(mutate(networks))
        print("Mutation finished", end=' ')
        for _ in range(population_size):
            car = out_q.get()
            to_grid(car)
        networks = []
        filled = 0
        for p in car_grid.ravel():
            if not p == 0:
                networks.append((p.network, p.alive_counter, p.avg_acc, p.avg_brk, p.max_lateral, p.distance, p.fuel, p.score))
                filled += 1
        print("Gen:", gen, "Best score:", best_car.score, 'Filled spaces in grid:', filled, '/',
              acc_pedal * brk_pedal * avg_sp * lateral * fc_c, 'Improved:', improv)
        np.save('grid.npy', np.asarray(networks), allow_pickle=True)
        gen += 1
        if gen > 200:
            for _ in range(process_count):
                in_q.put(None)
            break
