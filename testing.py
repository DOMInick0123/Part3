import math
from multiprocessing import Process, Queue

import numpy as np

from Car import Car
import time

# constants
physics_engine = 5
acc_pedal = 6
brk_pedal = 6
lateral = 6
fc_c = 6
avg_dis = 6
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
    in_q = Queue()
    out_q = Queue()
    processes = []
    for _ in range(4):
        x = Process(target=thread_function, args=(in_q, out_q))
        x.start()
        processes.append(x)
    networks = np.load('grid.npy', allow_pickle=True)
    for network in networks:
        in_q.put(Car(network[0]))
    for _ in range(len(networks)):
        car = out_q.get()
        to_grid(car)
    for _ in range(4):
        in_q.put(None)
    networks = []
    for p in car_grid.ravel():
        if not p == 0:
            networks.append((p.network, p.alive_counter, p.avg_acc, p.avg_brk, p.max_lateral, p.distance, p.fuel, p.score, p.mid))
    np.save('grid.npy', np.asarray(networks), allow_pickle=True)
