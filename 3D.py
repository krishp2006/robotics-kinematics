import numpy as np
import math
import matplotlib.pyplot as plt


L1 = 5


def transformation_matrix_z(theta, length):

    return np.array([
        [math.cos(theta), -math.sin(theta), 0, length * math.cos(theta)],
        [math.sin(theta),  math.cos(theta), 0, length * math.sin(theta)],
        [0,                0,               1, 0],
        [0,                0,               0, 1]
    ])


theta1 = math.radians(45)

T01 = transformation_matrix_z(
    theta1,
    L1
)


origin = np.array([
    [0],
    [0],
    [0],
    [1]
])


joint1 = T01 @ origin


x = [
    0,
    joint1[0, 0]
]

y = [
    0,
    joint1[1, 0]
]

z = [
    0,
    joint1[2, 0]
]


fig = plt.figure()

ax = fig.add_subplot(
    111,
    projection='3d'
)


ax.plot(
    x,
    y,
    z,
    'o-',
    linewidth=3
)


ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 10)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_title("3D Robot")


plt.show()