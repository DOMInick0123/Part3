import math
import threading
import time
import numpy as np
from random import choice, random, gauss
from Car import Car

# constants
physics_engine = 5
acc_pedal = 16
brk_pedal = 8
avg_sp = 8
lateral = 6
fc_c = 16
best_car = Car()
population_size = 800
thread_count = 80
car_counter = 0
lock = threading.Lock()
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


def thread_function():
    while 1:
        global car_counter
        lock.acquire()
        if car_counter == population_size:
            lock.release()
            break
        thread_player = cars[car_counter]
        car_counter += 1
        lock.release()
        while thread_player.alive:
            thread_player.think()
            for j in range(physics_engine):
                thread_player.update_car(0.01)


if __name__ == '__main__':
    gen = 1
    improv = 0
    cars = []
    networks = []
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
        cars = []
        for i in range(population_size//(2*gen)):
            cars.append(Car())
        while len(cars) < population_size:
            cars.append(mutate(networks))
        print("Mutation finished", end=' ')
        threads = []
        for i in range(thread_count):
            x = threading.Thread(target=thread_function)
            x.start()
            threads.append(x)
        cou = 0
        while len(threads) > 0:
            for thread in threads:
                if not thread.is_alive():
                    threads.remove(thread)
            time.sleep(10)
            cou += 1
            if cou > 60:
                for p in cars:
                    if p.alive:
                        np.save('error.npy', np.asarray(p.network), allow_pickle=True)
                        exit(1)
        for player in cars:
            to_grid(player)
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
        if gen > 100:
            break
