import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


def ellipsoid_eq(params, x, y, z):
    A, B, C, D, E, F, G, H, I, J = params
    return A * x**2 + B * y**2 + C * z**2 + D * x * y + E * x * z + F * y * z + G * x + H * y + I * z + J


def ellipsoid_residual(params, points):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    eq = ellipsoid_eq(params, x, y, z)

    distances = eq ** 2
    return np.sum(distances)


def fit_quadratic(points, x_start, x_end, y_start, y_end, z_start, z_end, sample):
    initial_params = np.random.rand(10)

    result = minimize(ellipsoid_residual, initial_params, args=(points,), method='L-BFGS-B')
    params = result.x
    print(params)

    x = np.linspace(x_start, x_end, sample)
    y = np.linspace(y_start, y_end, sample)
    x, y = np.meshgrid(x, y)
    p = []
    A, B, C, D, E, F, G, H, I, J = params
    for i in np.arange(x.shape[0]):
        for j in np.arange(x.shape[1]):
            a = C
            b = E*x[i, j]+F*y[i, j]+I
            c = A*x[i, j]**2+B*y[i, j]**2+D*x[i, j]*y[i, j]+G*x[i, j]+H*y[i, j]+J
            if a != 0:
                delta = b*b-4*a*c
                if delta >= 0:
                    z = (-b+np.sqrt(delta))/(2*a)
                    if z_start < z < z_end:
                        p.append([x[i, j], y[i, j], z])
                    z = (-b-np.sqrt(delta))/(2*a)
                    if z_start < z < z_end:
                        p.append([x[i, j], y[i, j], z])
            elif b != 0:
                z = -c/b
                if z_start < z < z_end:
                    p.append([x[i, j], y[i, j], z])
    p = np.asarray(p)
    return p


# Example usage with noisy points around an ellipsoid
points = np.random.rand(100, 3)  # Random noisy 3D points
for point in points:
    point[2] = 0.5+0.25*np.sin(3*point[0])+0.25*np.cos(3*point[1])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2])
p = fit_quadratic(points, 0, 1, 0, 1, 0, 1, 20)
ax.scatter(p[:, 0], p[:, 1], p[:, 2])

plt.show()