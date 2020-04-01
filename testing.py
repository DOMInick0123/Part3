from multiprocessing import Process, Queue
from Car import Car
import time

# constants
physics_engine = 5
population_size = 500


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
    car = Car()
    in_q = Queue()
    out_q = Queue()
    for i in range(1, 6, 1):
        processes = []
        for _ in range(i):
            x = Process(target=thread_function, args=(in_q, out_q))
            x.start()
            processes.append(x)
        before = time.time()
        for _ in range(population_size):
            in_q.put(car.copy())
        for _ in range(population_size):
            car = out_q.get()
        after = time.time()
        for _ in range(i):
            in_q.put(None)
        print(i, after-before)

