import time

from Car import Car
import math
from multiprocessing import Process, Queue
import numpy as np
from random import choice, random, gauss


def thread_function(in_q, out_q):
    while 1:
        car = in_q.get()
        if car is None:
            break
        while car.alive:
            car.think()
            for j in range(5):
                car.update_car(0.01)
        out_q.put(car)

if __name__ == '__main__':
    base = Car()
    in_q = Queue()
    out_q = Queue()
    for i in range(4, 17, 1):
        cars = []
        processes = []
        for _ in range(i):
            x = Process(target=thread_function, args=(in_q, out_q))
            x.start()
            processes.append(x)
        before = time.time()
        for _ in range(1000):
            in_q.put(base.copy())
        for _ in range(1000):
            car = out_q.get()
        after = time.time()
        print(i, after-before)
        for _ in range(i):
            in_q.put(None)
