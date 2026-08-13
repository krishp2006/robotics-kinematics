import numpy as np
import math
import matplotlib.pyplot as plt

# Robot dimensions

L1 = 5
L2 = 8
L3 = 6


# Rotation around Z

def rotation_z(theta):

    return np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta),  math.cos(theta), 0],
        [0,                0,               1]
    ])


# Rotation around Y

def rotation_y(theta):

    return np.array([
        [ math.cos(theta), 0, math.sin(theta)],
        [ 0,                1, 0              ],
        [-math.sin(theta), 0, math.cos(theta)]
    ])


# Create a 4x4 transformation matrix

def transformation_matrix(rotation, translation):

    T = np.eye(4)

    T[:3, :3] = rotation
    T[:3, 3] = translation

    return T


# Forward kinematics

def forward_kinematics(theta1, theta2, theta3):

    # Base rotation

    R1 = rotation_z(theta1)

    T01 = transformation_matrix(
        R1,
        [0, 0, L1]
    )

    # Shoulder rotation

    R2 = rotation_y(theta2)

    T12 = transformation_matrix(
        R2,
        [L2, 0, 0]
    )

    # Elbow rotation

    R3 = rotation_y(theta3)

    T23 = transformation_matrix(
        R3,
        [L3, 0, 0]
    )

    # Combine transformations

    T02 = T01 @ T12

    T03 = T02 @ T23

    # Origin of each frame

    origin = np.array([
        [0],
        [0],
        [0],
        [1]
    ])

    joint1 = T01 @ origin
    joint2 = T02 @ origin
    end_effector = T03 @ origin

    return (
        joint1,
        joint2,
        end_effector,
        T01,
        T02,
        T03
    )


# Initial angles

theta1 = math.radians(45)
theta2 = math.radians(30)
theta3 = math.radians(-20)


joint1, joint2, end_effector, T01, T02, T03 = forward_kinematics(
    theta1,
    theta2,
    theta3
)


# Extract positions

x = [
    0,
    joint1[0, 0],
    joint2[0, 0],
    end_effector[0, 0]
]

y = [
    0,
    joint1[1, 0],
    joint2[1, 0],
    end_effector[1, 0]
]

z = [
    0,
    joint1[2, 0],
    joint2[2, 0],
    end_effector[2, 0]
]


# Plot

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


# Plot settings

ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(0, 20)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_title("3-DOF Robot - 3D Forward Kinematics")

ax.set_box_aspect([1, 1, 1])

ax.grid(True)

plt.show()