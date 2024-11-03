import numpy as np
import matplotlib.pyplot as plt

points = np.random.rand(100, 3)
for point in points:
    point[2] = 1-32*(point[0]-0.5)**2-16*(point[1]-0.5)**2

with open('SurfaceFitting/points.txt', 'w') as f:
    for point in points:
        f.writelines(str(point[0])+' '+str(point[1])+' '+str(point[2])+'\n')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2])

with open('SurfaceFitting/ez_param.txt') as f:
    line = f.readline()
    term = line.split(' ')
    J, G, H, I, A, D, E, B, F, C, _ = term
    A = float(A)
    B = float(B)
    C = float(C)
    D = float(D)
    E = float(E)
    F = float(F)
    G = float(G)
    H = float(H)
    I = float(I)
    J = float(J)
    x = np.linspace(0, 1, 10)
    y = np.linspace(0, 1, 10)
    x, y = np.meshgrid(x, y)
    p = []
    for i in np.arange(x.shape[0]):
        for j in np.arange(x.shape[1]):
            a = C
            b = E * x[i, j] + F * y[i, j] + I
            c = A * x[i, j] ** 2 + B * y[i, j] ** 2 + D * x[i, j] * y[i, j] + G * x[i, j] + H * y[i, j] + J
            if a != 0:
                delta = b * b - 4 * a * c
                if delta >= 0:
                    z = (-b + np.sqrt(delta)) / (2 * a)
                    p.append([x[i, j], y[i, j], z])
                    #z = (-b - np.sqrt(delta)) / (2 * a)
                    p.append([x[i, j], y[i, j], z])
            elif b != 0:
                z = -c / b
                p.append([x[i, j], y[i, j], z])
            else:
                print('fuck')
    p = np.asarray(p)
    if len(p) > 0:
        ax.scatter(p[:, 0], p[:, 1], p[:, 2])

plt.show()