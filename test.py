import random

import numpy as np
import matplotlib.pyplot as plt

rou = 1e5

def sample_exponential(u, a):
    return -np.log(1-u)/a

def gaussian_distribution(x, miu, sigma):
    return np.exp(-(x-miu)**2/(2*sigma**2))/np.sqrt(2*np.pi*sigma**2)

def density(x, sigma):
    return gaussian_distribution(x, 0, sigma)/sigma

def eval_transmittance(x0, t_max, sigma):
    max_density = gaussian_distribution(0, 0, sigma)/sigma
    n_samples = 1024
    result = 0
    for i in np.arange(n_samples):
        t = 0
        while True:
            t += sample_exponential(random.random(), max_density)
            if t >= t_max:
                result += 1
                break
            x = x0+t
            density_at_t = density(x, sigma)
            if density_at_t/max_density > random.random():
                break
    return result/n_samples

def sample_distance(x0, t_max, sigma):
    max_density = gaussian_distribution(0, 0, sigma)/sigma
    t = 0
    while True:
        t += sample_exponential(random.random(), max_density)
        if t >= t_max:
            t = t_max
            break
        x = x0+t
        density_at_t = density(x, sigma)
        if density_at_t/max_density > random.random():
            break
    return t


sigma = 1
# print(eval_transmittance(-3*sigma, 6*sigma, sigma))
samples = 1024
d = 0
for i in np.arange(samples):
    d += sample_distance(-3*sigma, 6*sigma, sigma)
d /= samples
print(eval_transmittance(-3*sigma, 6*sigma, sigma))
print(d)
print(eval_transmittance(3*sigma-d, d, sigma))