import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义三次多项式的系数
def quadric_surface(x, y, coeffs):
    a, b, c, d, e, f, g, h, i, j = coeffs
    # 计算对应的z值
    A = c
    B = f * y + e * x + i
    C = a * x**2 + b * y**2 + d * x * y + g * x + h * y + j
    # 解一元二次方程 Az^2 + Bz + C = 0
    discriminant = B**2 - 4*A*C
    z1 = np.where(discriminant >= 0, (-B + np.sqrt(discriminant)) / (2*A), np.nan)
    z2 = np.where(discriminant >= 0, (-B - np.sqrt(discriminant)) / (2*A), np.nan)
    return z1, z2

# 系数
coeffs = [-0.23827, -0.18457, -0.77251, 0.0743, 0.2988, 0.04664, -0.12364, 0.12645, 0.42087, -0.08364]

coeffs[9] += 3*1.32146e-6
# 定义更小的范围
x = np.linspace(-0.03121, -0.01129, 100)
y = np.linspace(0.36589, 0.38581, 100)
X, Y = np.meshgrid(x, y)
Z1, Z2 = quadric_surface(X, Y, coeffs)

# 绘图
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# 只绘制有效区域
ax.plot_surface(X, Y, Z1, cmap='viridis', alpha=0.6, linewidth=0)
ax.plot_surface(X, Y, Z2, cmap='plasma', alpha=0.6, linewidth=0)

ax.set_title("Zoomed Region of the Quadratic Surface")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_zlim(0.27358, 0.29349)

plt.tight_layout()
plt.show()
