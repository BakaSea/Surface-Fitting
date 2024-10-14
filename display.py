import numpy as np
import matplotlib.pyplot as plt
import trimesh

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
with open('SurfaceFitting/param.txt') as f:
    for line in f.readlines():
        term = line.split(' ')
        _, x0, y0, z0, _, x1, y1, z1, _, J, G, H, I, A, D, E, B, F, C = term
        x0 = float(x0)
        y0 = float(y0)
        z0 = float(z0)
        x1 = float(x1)
        y1 = float(y1)
        z1 = float(z1)
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
        x = np.linspace(x0, x1, 10)
        y = np.linspace(y0, y1, 10)
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
                        if z0 <= z <= z1:
                            p.append([x[i, j], y[i, j], z])
                        z = (-b - np.sqrt(delta)) / (2 * a)
                        if z0 <= z <= z1:
                            p.append([x[i, j], y[i, j], z])
                elif b != 0:
                    z = -c / b
                    if z0 <= z < z1:
                        p.append([x[i, j], y[i, j], z])
        p = np.asarray(p)
        if len(p) > 0:
            ax.scatter(p[:, 0], p[:, 1], p[:, 2])

mesh = trimesh.load('example_data/bunny_zip.obj')
vertices = mesh.vertices
faces = mesh.faces
#ax.plot_trisurf(vertices[:, 0], vertices[:, 1], faces, vertices[:, 2], alpha=0.5)

plt.show()