import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Robot dimensions

L1 = 5
L2 = 8
L3 = 6

# Transformation matrix

def transformation_matrix(theta, length):

    return np.array([
        [math.cos(theta), -math.sin(theta), length * math.cos(theta)],
        [math.sin(theta),  math.cos(theta), length * math.sin(theta)],
        [0,                0,               1]
    ])


# Forward kinematics

def forward_kinematics(theta1, theta2, theta3):

    T01 = transformation_matrix(theta1, L1)

    T12 = transformation_matrix(theta2, L2)

    T23 = transformation_matrix(theta3, L3)

    T02 = T01 @ T12

    T03 = T02 @ T23

    origin = np.array([
        [0],
        [0],
        [1]
    ])

    joint1 = T01 @ origin
    joint2 = T02 @ origin
    end_effector = T03 @ origin

    return (
        joint1[0, 0],
        joint1[1, 0],
        joint2[0, 0],
        joint2[1, 0],
        end_effector[0, 0],
        end_effector[1, 0],
        T01,
        T02,
        T03
    )


# Check joint limits

def check_joint_limits(theta1, theta2, theta3):

    warnings = []

    angles = [
        math.degrees(theta1),
        math.degrees(theta2),
        math.degrees(theta3)
    ]

    limits = [
        (-180, 180),
        (-90, 90),
        (-90, 90)
    ]

    for i in range(3):

        angle = angles[i]
        minimum, maximum = limits[i]

        if angle < minimum or angle > maximum:

            warnings.append(
                f"Joint {i + 1} outside limits!"
            )

    return warnings


# Initial angles

theta1 = math.radians(45)
theta2 = math.radians(20)
theta3 = math.radians(20)

(
    x1, y1,
    x2, y2,
    x3, y3,
    T01, T02, T03
) = forward_kinematics(
    theta1,
    theta2,
    theta3
)


# Plot

fig, ax = plt.subplots()

plt.subplots_adjust(
    bottom=0.30
)

line, = ax.plot(
    [0, x1, x2, x3],
    [0, y1, y2, y3],
    'o-',
    linewidth=3
)

ax.set_xlim(-25, 25)
ax.set_ylim(-25, 25)

ax.set_aspect('equal')

ax.grid(True)

ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.set_title("3-DOF Robot - 2D Forward Kinematics")


# Debug information

debug_text = ax.text(
    0.02,
    0.98,
    "",
    transform=ax.transAxes,
    verticalalignment="top",
    horizontalalignment="left",
    fontsize=9,
    family="monospace"
)


# Sliders

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


# Update debug information

def update_debug(
    theta1,
    theta2,
    theta3,
    x1,
    y1,
    x2,
    y2,
    x3,
    y3,
    warnings
):

    if warnings:
        status = "STATUS: WARNING"
    else:
        status = "STATUS: OK"

    debug_text.set_text(
        f"""
{status}

JOINT ANGLES
----------------
Theta 1: {math.degrees(theta1):7.2f}°
Theta 2: {math.degrees(theta2):7.2f}°
Theta 3: {math.degrees(theta3):7.2f}°

JOINT POSITIONS
----------------
Joint 1: ({x1:6.2f}, {y1:6.2f})
Joint 2: ({x2:6.2f}, {y2:6.2f})
End:     ({x3:6.2f}, {y3:6.2f})

WARNINGS
----------------
{chr(10).join(warnings) if warnings else "None"}
"""
    )


# Update simulation

def update(val):

    theta1 = math.radians(slider_theta1.val)
    theta2 = math.radians(slider_theta2.val)
    theta3 = math.radians(slider_theta3.val)

    warnings = check_joint_limits(
        theta1,
        theta2,
        theta3
    )

    (
        x1, y1,
        x2, y2,
        x3, y3,
        T01, T02, T03
    ) = forward_kinematics(
        theta1,
        theta2,
        theta3
    )

    line.set_data(
        [0, x1, x2, x3],
        [0, y1, y2, y3]
    )

    update_debug(
        theta1,
        theta2,
        theta3,
        x1,
        y1,
        x2,
        y2,
        x3,
        y3,
        warnings
    )

    fig.canvas.draw_idle()


slider_theta1.on_changed(update)
slider_theta2.on_changed(update)
slider_theta3.on_changed(update)

update(None)

plt.show()