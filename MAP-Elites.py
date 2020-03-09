import math
import threading
import time
import numpy as np
from random import randrange
from PlayerTesting import Player

# constants
physics_engine = 5
acc_pedal = 10
brk_pedal = 10
dis_c = 10
lateral = 10
fc_c = 10

best_player = Player()
thread_count = 1
population_size = 1
players = []
#networks = np.load('grid.npy', allow_pickle=True)
#for network in networks:
    #players.append(Player(network))
while len(players) < population_size:
    players.append(Player())
player_counter = len(players)
lock = threading.Lock()
player_grid = np.zeros((acc_pedal, brk_pedal, dis_c, lateral, fc_c), dtype=Player)


def thread_function():
    while 1:
        global player_counter
        lock.acquire()
        if player_counter == 0:
            lock.release()
            break
        thread_player = players[player_counter -1]
        player_counter -= 1
        lock.release()
        while thread_player.alive:
            thread_player.think()
            for j in range(physics_engine):
                thread_player.update_player(0.01)


def get_random_player():
    p1 = player_grid[randrange(acc_pedal)][randrange(brk_pedal)][randrange(dis_c)][randrange(lateral)]
    while p1 == 0:
        p1 = player_grid[randrange(acc_pedal)][randrange(brk_pedal)][randrange(dis_c)][randrange(lateral)]
    return p1


if __name__ == '__main__':
    gen = 0
    f = open("progress.txt", "a+")
    while 1:
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
        player_counter = population_size
        for player in players:
            if player.score > best_player.score:
                best_player = player
            acc = int(acc_pedal/2*(math.tanh(5*(player.avg_acc/player.alive_counter*physics_engine-0.6))+1))
            brk = int(brk_pedal/2*(math.tanh(5*(player.avg_brk/player.alive_counter*physics_engine-0.3))+1))
            dis = int(dis_c/2*(math.tanh(5*player.avg_dis/player.alive_counter*physics_engine)+1))
            lat = min(int(player.max_lateral/20000*lateral), 9)
            fc = player.distance/(100.-player.fuel)
            grid_player = player_grid[acc, brk, dis, lat, fc]
            if grid_player == 0 or player.score > grid_player.score:
                player_grid[acc, brk, dis, lat, fc] = player
        np.save('bp.npy', np.asarray(best_player.network), allow_pickle=True)
        networks = []
        filled = 0
        for p in player_grid.ravel():
            if not p == 0:
                networks.append(p.network)
                filled += 1
        print("Gen:", gen, "Best score:", best_player.score, 'Filled spaces in grid:', filled, '/', acc_pedal*brk_pedal*dis_c*lateral*fc_c)
        f.write("Gen: "+str(gen)+", Best score: "+str(best_player.score)+" Filled spaces in grid: "+str(filled)+"/"+str(acc_pedal*brk_pedal*dis_c*lateral*fc_c))
        np.save('grid.npy', np.asarray(networks), allow_pickle=True)
        gen += 1
        if gen == 300:
            break
        new_players = []
        while len(new_players) < population_size:
            new_players.append(get_random_player().crossover(get_random_player()))
        players = new_players

