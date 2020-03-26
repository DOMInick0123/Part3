import math
import threading
import time
import numpy as np
from random import random, gauss, choice
from Car import Car

# constants
physics_engine = 5
acc_pedal = 16
brk_pedal = 16
dis_c = 6
lateral = 6
fc_c = 16
best_car = Car()
thread_count = 128
population_size = 1024
car_counter = 0
lock = threading.Lock()
car_grid = np.zeros((acc_pedal, brk_pedal, dis_c, lateral, fc_c), dtype=Car)


def mutate(grid):
    nn = []
    p1 = choice(grid)[0]
    p2 = choice(grid)[0]
    for i in range(len(p1)):
        weights = []
        bias = []
        for j in range(len(p1[i][0])):
            row = []
            for k in range(len(p1[i][0][j])):
                if random() < 0.5:
                    row.append(p1[i][0][j][k])
                else:
                    row.append(p2[i][0][j][k])
            weights.append(row)
            if random() < 0.5:
                bias.append(p1[i][1][j])
            else:
                bias.append(p2[i][1][j])
        nn.append((weights, bias))
    for layer in nn:
        for i in range(len(layer[0])):
            for j in range(len(layer[0][i])):
                rand = random()
                if rand < 0.5:
                    layer[0][i][j] += gauss(0, 1) / 50
                    layer[0][i][j] = min(layer[0][i][j], 1)
                    layer[0][i][j] = max(layer[0][i][j], -1)
                elif rand > 0.9:
                    layer[0][i][j] = random() * 2 - 1
        for i in range(len(layer[1])):
            w = len(layer[0][1])
            rand = random()
            if rand < 0.5:
                layer[1][i] += gauss(0, 1) / 50 * w
                layer[1][i] = min(layer[1][i], w)
                layer[1][i] = max(layer[1][i], -w)
            elif rand > 0.9:
                layer[1][i] = w*(random() * 2 - 1)
    p = Car(nn)
    return p


def to_grid(car):
    global best_car, improv
    acc = int(
        acc_pedal / 2 * (math.tanh(8 * (car.avg_acc / car.alive_counter * physics_engine - 0.8)) + 1))
    brk = int(
        brk_pedal / 2 * (math.tanh(8 * (car.avg_brk / car.alive_counter * physics_engine - 0.15)) + 1))
    dis = int((dis_c - 1) / 2 * (math.tanh(10 * car.avg_dis / car.alive_counter * physics_engine) + 1))
    lat = min(int(car.max_lateral / 20000 * lateral), lateral - 1)
    try:
        fc = min(int(math.tanh(car.distance / (100. - car.fuel) / 1000) * fc_c), fc_c - 1)
    except ZeroDivisionError:
        fc = 0
    grid_car = car_grid[acc, brk, dis, lat, fc]
    if grid_car == 0 or car.score > grid_car.score:
        car_grid[acc, brk, dis, lat, fc] = car
        if car.score > best_car.score:
            best_car = car
        improv += 1


def thread_function():
    while 1:
        global car_counter
        lock.acquire()
        if car_counter >= population_size:
            lock.release()
            break
        thread_car = cars[car_counter]
        car_counter += 1
        lock.release()
        while thread_car.alive:
            thread_car.think()
            for j in range(physics_engine):
                thread_car.update_car(0.01)


if __name__ == '__main__':
    gen = 0
    improv = 0
    f = open("progress.txt", "a+")
    cars = []
    try:
        networks = np.load('grid.npy', allow_pickle=True)
    except IOError:
        while len(cars) < population_size:
            cars.append(Car())
    else:
        for network in networks:
            p = Car(network[0])
            p.alive_counter = network[1]
            p.avg_acc = network[2]
            p.avg_brk = network[3]
            p.avg_dis = network[4]
            p.max_lateral = network[5]
            p.distance = network[6]
            p.fuel = network[7]
            p.score = network[8]
            to_grid(p)
        while len(cars) < population_size:
            cars.append(Car())
    while 1:
        threads = []
        for i in range(thread_count):
            x = threading.Thread(target=thread_function)
            x.start()
            threads.append(x)
        thread_function()
        while len(threads) > 0:
            for thread in threads:
                if not thread.is_alive():
                    threads.remove(thread)
            time.sleep(5)
        car_counter = 0
        improv = 0
        for player in cars:
            to_grid(player)
        np.save('bp.npy', np.asarray(best_car.network), allow_pickle=True)
        networks = []
        filled = 0

        for p in car_grid.ravel():
            if not p == 0:
                networks.append((p.network, p.alive_counter, p.avg_acc, p.avg_brk, p.avg_dis, p.max_lateral, p.distance, p.fuel, p.score))
                filled += 1
        print("Gen:", gen, "Best score:", best_car.score, 'Filled spaces in grid:', filled, '/',
              acc_pedal * brk_pedal * dis_c * lateral * fc_c, 'Improved:', improv)
        f.write("Gen: " + str(gen) + ", Best score: " + str(best_car.score) + " Filled spaces in grid: " + str(
            filled) + "/" + str(acc_pedal * brk_pedal * dis_c * lateral * fc_c) + '\n')
        np.save('grid.npy', np.asarray(networks), allow_pickle=True)
        gen += 1
        if gen == 300:
            break
        new_cars = []
        while len(new_cars) < population_size:
            new_cars.append(mutate(networks))
        cars = new_cars
