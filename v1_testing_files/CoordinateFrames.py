import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


# Robot dimensions
L1 = 5
L2 = 8
L3 = 6


# Transformation matrix
def transformation_matrix(theta, x, y):

    return np.array([
        [math.cos(theta), -math.sin(theta), x],
        [math.sin(theta),  math.cos(theta), y],
        [0,                0,               1]
    ])


# Forward Kinematics
def forward_kinematics(theta1, theta2, theta3):

    # Base → Joint 1
    T01 = transformation_matrix(
        theta1,
        L1,
        0
    )

    # Joint 1 → Joint 2
    T12 = transformation_matrix(
        theta2,
        L2,
        0
    )

    # Joint 2 → End Effector
    T23 = transformation_matrix(
        theta3,
        L3,
        0
    )

    # Combine transformations
    T02 = T01 @ T12
    T03 = T02 @ T23

    # Origin of the final frame
    origin = np.array([
        [0],
        [0],
        [1]
    ])

    # End effector position
    end_effector = T03 @ origin

    # Get Joint 1 position
    origin = np.array([
        [0],
        [0],
        [1]
    ])

    joint1 = T01 @ origin
    joint2 = T02 @ origin

    return (
        joint1[0, 0],
        joint1[1, 0],
        joint2[0, 0],
        joint2[1, 0],
        end_effector[0, 0],
        end_effector[1, 0]
    )


# Initial angles
theta1 = math.radians(45)
theta2 = math.radians(20)
theta3 = math.radians(20)


x1, y1, x2, y2, x3, y3 = forward_kinematics(
    theta1,
    theta2,
    theta3
)


# Plot
fig, ax = plt.subplots()

plt.subplots_adjust(
    bottom=0.30
)


# Draw robot

line, = ax.plot(
    [0, x1, x2, x3],
    [0, y1, y2, y3],
    'o-',
    linewidth=3
)


# Plot settings
ax.set_xlim(-25, 25)
ax.set_ylim(-25, 25)

ax.set_aspect('equal')

ax.grid(True)

ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.set_title("3-DOF Robot - Forward Kinematics")

# Debug information
debug_text = ax.text(
    0.02,
    0.98,
    "",
    transform=ax.transAxes,
    verticalalignment="top",
    fontsize=9,
    family="monospace"
)

# Slider 1
ax_theta1 = plt.axes([
    0.20,
    0.20,
    0.65,
    0.03
])

slider_theta1 = Slider(
    ax_theta1,
    'Theta 1',
    -180,
    180,
    valinit=45
)


# Slider 2
ax_theta2 = plt.axes([
    0.20,
    0.13,
    0.65,
    0.03
])

slider_theta2 = Slider(
    ax_theta2,
    'Theta 2',
    -180,
    180,
    valinit=20
)


# Slider 3
ax_theta3 = plt.axes([
    0.20,
    0.06,
    0.65,
    0.03
])

slider_theta3 = Slider(
    ax_theta3,
    'Theta 3',
    -180,
    180,
    valinit=20
)


# Update function
def update(val):

    # Get slider values
    theta1 = math.radians(
        slider_theta1.val
    )

    theta2 = math.radians(
        slider_theta2.val
    )

    theta3 = math.radians(
        slider_theta3.val
    )


    # Calculate FK
    x1, y1, x2, y2, x3, y3 = forward_kinematics(
        theta1,
        theta2,
        theta3
    )


    # Update robot
    line.set_data(
        [0, x1, x2, x3],
        [0, y1, y2, y3]
    )


    # Redraw
    fig.canvas.draw_idle()


# Connect sliders
slider_theta1.on_changed(update)
slider_theta2.on_changed(update)
slider_theta3.on_changed(update)


# Show
plt.show()