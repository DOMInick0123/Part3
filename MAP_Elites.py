import math
from multiprocessing import Process, Queue
import numpy as np
from random import choice, random
from Car import Car
from copy import deepcopy
# main class of the algorithm
# constants
physics_engine = 5
acc_pedal = 6
brk_pedal = 6
lateral = 6
fc_c = 6
avg_dis = 6
population_size = 1024
process_count = 16


# function which pick a random individual and create an offspring by using mutation and crossover
def mutate(grid):
    if len(grid) == 0:
        return Car()
    car1 = deepcopy(choice(grid)[0])
    car2 = deepcopy(choice(grid)[0])
    nn = []
    r = 0.992
    # crossover
    for i in range(len(car1)):
        weights = []
        bias = []
        for j in range(len(car1[i][0])):
            row = []
            for k in range(len(car1[i][0][j])):
                if random() < r:
                    row.append(car1[i][0][j][k])
                else:
                    row.append(car2[i][0][j][k])
            weights.append(row)
            if random() < r:
                bias.append(car1[i][1][j])
            else:
                bias.append(car2[i][1][j])
        nn.append((weights, bias))
    # mutation
    for layer in nn:
        for i in range(len(layer[0])):
            for j in range(len(layer[0][i])):
                if random() > r:
                    layer[0][i][j] = random() * 2 - 1
        for i in range(len(layer[1])):
            if random() > r:
                layer[1][i] = random() * 2 - 1
    return Car(nn)


# function which will place the individual into correct place in the grid
def to_grid(car):
    global best_car, improv
    if car.score > 0:
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
            if car.score > best_car.score:
                best_car = car
            improv += 1


# thread function for simulating cars on the queue
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
    best_car = Car()
    car_grid = np.zeros((acc_pedal, brk_pedal, lateral, fc_c, avg_dis), dtype=Car)
    improv = 0
    cars = []
    networks = []
    # initialisation of the helper processes
    in_q = Queue()
    out_q = Queue()
    processes = []
    for _ in range(process_count):
        x = Process(target=thread_function, args=(in_q, out_q))
        x.start()
        processes.append(x)
    # reading file of already generated individuals if they exist
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
    # reading the past progress from the algorithm
    try:
        progress = np.load('progress.npy', allow_pickle=True).tolist()
    except IOError:
        progress = []
    for gen in range(15000):
        car_counter = 0
        improv = 0
        for _ in range(population_size):
            in_q.put(mutate(networks))
        for _ in range(population_size):
            car = out_q.get()
            to_grid(car)
        # creation of flattened array of filled individuals from the grid
        networks = []
        filled = 0
        avg_score = 0
        for p in car_grid.ravel():
            if not p == 0:
                networks.append((p.network, p.alive_counter, p.avg_acc, p.avg_brk, p.max_lateral, p.distance, p.fuel, p.score, p.mid))
                filled += 1
                avg_score += p.score
        # printing progress
        print("Gen:", gen, "Best score:", best_car.score, 'Filled spaces in grid:', filled, '/',
              acc_pedal * brk_pedal * lateral * fc_c * avg_dis, 'Improved:', improv, 'Average score:', avg_score/filled)
        progress.append([best_car.score, filled, avg_score/filled])
        # saving grid and progress into files
        np.save('grid.npy', np.asarray(networks), allow_pickle=True)
        np.save('progress.npy', np.asarray(progress), allow_pickle=True)
    # stopping the helper processes
    for _ in range(process_count):
        in_q.put(None)
