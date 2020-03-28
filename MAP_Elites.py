import math
import threading
import time
import numpy as np
from random import choice
from Car import Car

# constants
physics_engine = 5
acc_pedal = 16
brk_pedal = 16
dis_c = 6
lateral = 6
fc_c = 16
best_car = Car()
thread_count = 2**3
population_size = 2**10
car_counter = 0
lock = threading.Lock()
car_grid = np.zeros((acc_pedal, brk_pedal, dis_c, lateral, fc_c), dtype=Car)


def sort_innovation_number(connection):
    return connection[4]


def sort_layer(connection):
    return connection[0]


def mutate(grid):
    car1 = choice(grid)
    car2 = choice(grid)
    new_car = car1.crossover(car2)
    new_car.add_node()
    new_car.add_connection()
    new_car.mutate()
    new_car.connections.sort(key=sort_layer)
    return new_car


def to_grid(car):
    global best_car, improv
    acc = int(
        acc_pedal / 2 * (math.tanh(8 * (car.avg_acc / car.alive_counter * physics_engine - 0.8)) + 1))
    brk = int(
        brk_pedal / 2 * (math.tanh(8 * (car.avg_brk / car.alive_counter * physics_engine - 0.15)) + 1))
    dis = int((dis_c - 1) / 2 * (math.tanh(10 * car.avg_dis / car.alive_counter * physics_engine) + 1))
    lat = min(int(car.max_lateral / 20000 * lateral), lateral - 1)
    if car.fuel == 100.:
        fc = 0
    else:
        fc = min(int(math.tanh(car.distance / (100. - car.fuel) / 1000) * fc_c), fc_c - 1)
    grid_car = car_grid[acc, brk, dis, lat, fc]
    if grid_car == 0 or car.score > grid_car.score:
        car_grid[acc, brk, dis, lat, fc] = car
        car.connections.sort(key=sort_innovation_number)
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
    cars = []
    try:
        networks = np.load('grid.npy', allow_pickle=True)
    except IOError:
        while len(cars) < population_size:
            cars.append(Car())
    else:
        grid = []
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
            grid.append(p)
        while len(cars) < population_size:
            cars.append(mutate(grid))
    while 1:
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
                        np.save('error.npy', np.asarray(p.connections), allow_pickle=True)
                        exit(1)
        car_counter = 0
        improv = 0
        for player in cars:
            to_grid(player)
        np.save('bp.npy', np.asarray(best_car.connections), allow_pickle=True)
        networks = []
        filled = 0
        grid = []
        for p in car_grid.ravel():
            if not p == 0:
                grid.append(p)
                networks.append((p.connections, p.alive_counter, p.avg_acc, p.avg_brk, p.avg_dis, p.max_lateral, p.distance, p.fuel, p.score))
                filled += 1
        print("Gen:", gen, "Best score:", best_car.score, 'Filled spaces in grid:', filled, '/',
              acc_pedal * brk_pedal * dis_c * lateral * fc_c, 'Improved:', improv)
        np.save('grid.npy', np.asarray(networks), allow_pickle=True)
        gen += 1
        if gen == 200:
            break
        new_cars = []
        while len(new_cars) < population_size:
            new_cars.append(mutate(grid))
        cars = new_cars
        print("Mutation finished", end=' ')
