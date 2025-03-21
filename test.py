import numpy as np

SAMPLES = 1

def f(q, x, y, z):
    return q[0]*x**2+q[1]*y**2+q[2]*z**2+q[3]*x*y+q[4]*x*z+q[5]*y*z+q[6]*x+q[7]*y+q[8]*z+q[9]

def df(q, x, y, z):
    return [
        2*q[0]*x+q[3]*y+q[4]*z+q[6],
        2*q[1]*y+q[3]*x+q[5]*z+q[7],
        2*q[2]*z+q[4]*x+q[5]*y+q[8]
    ]

def inBox(x, y, z, bmin, bmax):
    return bmin[0] <= x <= bmax[0] and bmin[1] <= y <= bmax[1] and bmin[2] <= z <= bmax[2]


# 0.729442 -1.02404 -1.51076 1.76856 0.0150725 -0.47164
# -0.164194 -0.315978 -0.16442 -0.0445044 0.000491966 0.0448935 0.357741 -0.543732 -0.272597 -0.586716
# 0.0185703 0.0219449

q = np.asarray([-0.164194, -0.315978, -0.16442, -0.0445044, 0.000491966, 0.0448935, 0.357741, -0.543732, -0.272597, -0.586716 ])
bmin = np.asarray([0.729442, -1.02404, -1.51076])
bmax = np.asarray([1.76856, 0.0150725, -0.47164])

X = np.random.rand(SAMPLES)
Y = np.random.rand(SAMPLES)
sum = 0
cap = bmax-bmin
pdf = 1/(cap[0]*cap[1])
for i in np.arange(SAMPLES):
    x = bmin[0]+X[i]*cap[0]
    y = bmin[1]+Y[i]*cap[1]
    x = 1.40190196
    y = -0.746403694
    a = q[2]
    b = q[4]*x+q[5]*y+q[8]
    c = f(q, x, y, 0)
    print(a, b, c)
    delta = b**2-4*a*c
    if delta > 0:
        if np.random.rand(1) < 0.5:
            z = (-b+np.sqrt(delta))/(2*a)
        else:
            z = (-b-np.sqrt(delta))/(2*a)
        print(x, y, z)
        if inBox(x, y, z, bmin, bmax):
            dfxyz = df(q, x, y, z)
            if dfxyz[2] != 0:
                fx = -dfxyz[0] / dfxyz[2]
                fy = -dfxyz[1] / dfxyz[2]
                print(dfxyz)
                print(fx, fy)
                sum += np.sqrt(1 + fx ** 2 + fy ** 2) / (pdf * 0.5)
                print(sum)
sum /= SAMPLES
print(sum)
print(cap[0]*cap[1])