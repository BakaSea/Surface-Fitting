import numpy as np
import matplotlib.pyplot as plt


def clip_tri(tri, bmin, bmax):
    res = tri
    for j in range(3):
        p = res
        size = len(p)
        res = []
        for i in range(size):
            e = p[(i+1)%size]-p[i]
            if e[j] != 0:
                t = (bmin[j]-p[i][j])/e[j]
                if 1e-5 < t < 1-(1e-5):
                    print(t)
                    res.append(p[i]+t*e)
            if p[(i+1)%size][j] >= bmin[j]:
                res.append(p[(i+1)%size])
        p = res
        size = len(p)
        res = []
        for i in range(size):
            e = p[(i+1)%size]-p[i]
            if e[j] != 0:
                t = (bmax[j]-p[i][j])/e[j]
                if 1e-5 < t < 1-(1e-5):
                    print(t)
                    res.append(p[i]+t*e)
            if p[(i+1)%size][j] <= bmax[j]:
                res.append(p[(i+1)%size])
    return res


tri = np.asarray([[-0.261364996, 0.678849936, -0.00768499961], [-0.261579990, 0.673545003, -0.0120099997], [-0.265699983, 0.675300002, -0.0140099991]])
print(tri[0], tri[1], tri[2])
points = np.asarray(clip_tri(tri, [-0.278826237, 0.550770044, -0.00768500566], [-0.0843024999, 0.743587494, 0.143057480]))
for i in range(len(points)):
    print(points[i])
faces = []
if len(points) >= 3:
    for i in np.arange(len(points)-2)+1:
        faces.append([0, i, i+1])


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.278826237, -0.0843024999)
ax.set_ylim(0.550770044, 0.743587494)
ax.set_zlim(-0.00768500566, 0.143057480)
ax.plot_trisurf(tri[:, 0], tri[:, 1], [[0, 1, 2]], tri[:, 2], alpha=0.5)
if len(points) >= 3:
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='orange')
    ax.plot_trisurf(points[:, 0], points[:, 1], faces, points[:, 2])

plt.show()
