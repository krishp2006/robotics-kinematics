import numpy as np
import math

def rotation_matrix(theta):
    return np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta),  math.cos(theta)]
    ])


L1 = 10

theta1 = math.radians(45)

R1 = rotation_matrix(theta1)

link1 = np.array([
    [L1],
    [0]
])

P1 = R1 @ link1

print(P1)