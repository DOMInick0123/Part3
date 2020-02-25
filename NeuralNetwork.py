import math
from random import random, gauss

def sigmoid(val):
    if val < -700:
        return 0
    return 1/(1+math.e**(-val))

class NeuralNetwork():
    def __init__(self):
        self.input_layer = 8
        if network is None:
            self.network = []
            # layers = (4, 4, 2)
            layers = (4, 2)
            for i in range(len(layers) - 1):
                weights = []
                bias = []
                for j in range(layers[i + 1]):
                    row = []
                    for k in range(layers[i]):
                        row.append(random() * 2 - 1)
                    bias.append(random() * 50 - 25)
                    weights.append(row)
                self.network.append((weights, bias))
        else:
            self.network = network
