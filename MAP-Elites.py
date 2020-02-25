import threading
import time
import numpy as np
from random import randrange
from PlayerTesting import Player

best_player = Player()
thread_count = 32
batch_size = 16
population_size = thread_count * batch_size
players = []
for i in range(population_size):
    players.append(Player())
player_counter = 0
lock = threading.Lock()
player_grid = np.zeros((101, 101), dtype=Player)


def thread_function():
    while 1:
        global player_counter
        lock.acquire()
        if player_counter == population_size:
            lock.release()
            break
        thread_player = players[player_counter]
        player_counter += 1
        lock.release()
        while thread_player.alive:
            thread_player.think()
            for j in range(5):
                thread_player.update_player(0.01)


if __name__ == '__main__':
    for j in range(4):
        threads = []
        for i in range(thread_count):
            x = threading.Thread(target=thread_function)
            x.start()
            threads.append(x)
        while len(threads) > 0:
            for thread in threads:
                if not thread.is_alive():
                    threads.remove(thread)
            time.sleep(5)
        for player in players:
            if player.score > best_player.score:
                best_player = player
            player.avg_acc /= player.alive_counter
            player.avg_brk /= player.alive_counter
            #player.avg_fc = player.distance/(100.-player.fuel)
            grid_player = player_grid[int(player.avg_acc*500)][int(player.avg_brk*500)]
            if grid_player == 0 or player.score > grid_player.score:
                player_grid[int(player.avg_acc*500)][int(player.avg_brk*500)] = player
        print(best_player.score)
        np.save('bp.npy', np.asarray(best_player.network), allow_pickle=True)
        new_players = []
        while len(new_players) < population_size:
            p1 = player_grid[randrange(101)][randrange(101)]
            while p1 == 0:
                p1 = player_grid[randrange(101)][randrange(101)]
            p2 = player_grid[randrange(101)][randrange(101)]
            while p2 == 0:
                p2 = player_grid[randrange(101)][randrange(101)]
            new_players.append(p1.crossover(p2))
        players = new_players

