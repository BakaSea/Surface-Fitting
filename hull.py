import numpy as np
import matplotlib.pyplot as plt
import trimesh

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

mesh = trimesh.load('example_data/bunny.obj')
vertices = mesh.vertices
faces = mesh.faces
#ax.plot_trisurf(vertices[:, 0], vertices[:, 1], faces, vertices[:, 2], alpha=0.5)

with open('SurfaceFitting/hull.txt') as f:
    for line in f.readlines():
        terms = line.split(' ')
        for i in range(len(terms)):
            terms[i] = float(terms[i])
        x0, y0, z0, x1, y1, z1, c0, c1, c2, a00, a10, a20, a01, a11, a21, a02, a12, a22 = terms
        center = np.asarray([[c0], [c1], [c2]])
        Q = np.asarray([[a00, a01, a02], [a10, a11, a12], [a20, a21, a22]])
        x = np.linspace(x0, x1, 10)
        y = np.linspace(y0, y1, 10)
        z = np.linspace(z0, z1, 10)
        p = []
        for i in np.arange(len(x)):
            for j in np.arange(len(y)):
                for k in np.arange(len(z)):
                    X = np.asarray([[x[i]], [y[j]], [z[k]]])
                    f = np.transpose(X-center)@np.transpose(Q)@Q@(X-center)
                    if f <= 1:
                        p.append([x[i], y[j], z[k]])
        p = np.asarray(p)
        if len(p) > 0:
            ax.scatter(p[:, 0], p[:, 1], p[:, 2])

plt.show()