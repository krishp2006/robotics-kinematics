import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button


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


# Create transformation matrix

def transformation_matrix(rotation, translation):

    T = np.eye(4)

    T[:3, :3] = rotation
    T[:3, 3] = translation

    return T


# Forward kinematics

def forward_kinematics(theta1, theta2, theta3):

    # J1: base rotation

    T01 = transformation_matrix(
        rotation_z(theta1),
        [0, 0, 0]
    )


    # Link from J1 to J2

    T12 = transformation_matrix(
        np.eye(3),
        [0, 0, L1]
    )


    # J2: shoulder rotation

    T23 = transformation_matrix(
        rotation_y(theta2),
        [0, 0, 0]
    )


    # Link from J2 to J3

    T34 = transformation_matrix(
        np.eye(3),
        [L2, 0, 0]
    )


    # J3: elbow rotation

    T45 = transformation_matrix(
        rotation_y(theta3),
        [0, 0, 0]
    )


    # Link from J3 to end effector

    T56 = transformation_matrix(
        np.eye(3),
        [L3, 0, 0]
    )


    # Combine transformations

    T02 = T01 @ T12

    T03 = T02 @ T23

    T04 = T03 @ T34

    T05 = T04 @ T45

    T06 = T05 @ T56


    # Origin

    origin = np.array([
        [0],
        [0],
        [0],
        [1]
    ])


    # Joint positions

    base = origin

    joint2 = T02 @ origin

    joint3 = T04 @ origin

    end_effector = T06 @ origin


    return (
        base,
        joint2,
        joint3,
        end_effector
    )


# Initial angles

theta1 = math.radians(45)
theta2 = math.radians(30)
theta3 = math.radians(-20)


base, joint2, joint3, end_effector = forward_kinematics(
    theta1,
    theta2,
    theta3
)


# Get positions

def get_positions(
    base,
    joint2,
    joint3,
    end_effector
):

    x = [
        base[0, 0],
        joint2[0, 0],
        joint3[0, 0],
        end_effector[0, 0]
    ]

    y = [
        base[1, 0],
        joint2[1, 0],
        joint3[1, 0],
        end_effector[1, 0]
    ]

    z = [
        base[2, 0],
        joint2[2, 0],
        joint3[2, 0],
        end_effector[2, 0]
    ]

    return x, y, z


x, y, z = get_positions(
    base,
    joint2,
    joint3,
    end_effector
)


# Plot

fig = plt.figure(
    figsize=(12, 7)
)


# 3D robot area

ax = fig.add_axes(
    [
        0.05,
        0.15,
        0.60,
        0.75
    ],
    projection='3d'
)


# Draw robot

line, = ax.plot(
    x,
    y,
    z,
    'o-',
    linewidth=3
)


# Plot settings

ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-15, 15)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_title(
    "3-DOF Robot - 3D Forward Kinematics"
)

ax.set_box_aspect([1, 1, 1])

ax.grid(True)


# Debug information

debug_text = fig.text(
    0.70,
    0.80,
    "",
    fontsize=10,
    family="monospace",
    verticalalignment="top"
)


# Base slider

ax_base = fig.add_axes([
    0.10,
    0.08,
    0.50,
    0.03
])

slider_base = Slider(
    ax_base,
    'J1 Base',
    -180,
    180,
    valinit=45
)


# Shoulder slider

ax_shoulder = fig.add_axes([
    0.10,
    0.045,
    0.50,
    0.03
])

slider_shoulder = Slider(
    ax_shoulder,
    'J2 Shoulder',
    -90,
    90,
    valinit=30
)


# Elbow slider

ax_elbow = fig.add_axes([
    0.10,
    0.01,
    0.50,
    0.03
])

slider_elbow = Slider(
    ax_elbow,
    'J3 Elbow',
    -135,
    135,
    valinit=-20
)


# Reset view button

ax_reset = fig.add_axes([
    0.70,
    0.10,
    0.20,
    0.05
])

reset_button = Button(
    ax_reset,
    'Reset View'
)


def reset_view(event):

    ax.view_init(
        elev=25,
        azim=-60
    )

    fig.canvas.draw_idle()


reset_button.on_clicked(reset_view)
# Update simulation

def update(val):

    theta1 = math.radians(
        slider_base.val
    )

    theta2 = math.radians(
        slider_shoulder.val
    )

    theta3 = math.radians(
        slider_elbow.val
    )


    base, joint2, joint3, end_effector = forward_kinematics(
        theta1,
        theta2,
        theta3
    )


    x, y, z = get_positions(
        base,
        joint2,
        joint3,
        end_effector
    )


    line.set_data_3d(
        x,
        y,
        z
    )


    debug_text.set_text(
        f"""
JOINTS

J1 Base:     {math.degrees(theta1):7.2f}°
J2 Shoulder: {math.degrees(theta2):7.2f}°
J3 Elbow:    {math.degrees(theta3):7.2f}°

POSITIONS

J1: ({base[0, 0]:6.2f}, {base[1, 0]:6.2f}, {base[2, 0]:6.2f})

J2: ({joint2[0, 0]:6.2f}, {joint2[1, 0]:6.2f}, {joint2[2, 0]:6.2f})

J3: ({joint3[0, 0]:6.2f}, {joint3[1, 0]:6.2f}, {joint3[2, 0]:6.2f})

END: ({end_effector[0, 0]:6.2f}, {end_effector[1, 0]:6.2f}, {end_effector[2, 0]:6.2f})
"""
    )


    fig.canvas.draw_idle()


# Connect sliders

slider_base.on_changed(update)
slider_shoulder.on_changed(update)
slider_elbow.on_changed(update)


# Initialize

update(None)



plt.show()