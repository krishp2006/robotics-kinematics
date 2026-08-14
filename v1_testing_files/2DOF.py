import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons

L1 = 10
L2 = 8



# Forward Kinematics
def get_arm_positions(theta1, theta2):

    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return x1, y1, x2, y2



# Inverse Kinematics
def inverse_kinematics(x, y, elbow_up=True):

    # Distance from base to target
    r = math.sqrt(x**2 + y**2)

    # Check if target is reachable
    if r > L1 + L2 or r < abs(L1 - L2):
        return None

    # Law of Cosines
    cos_theta2 = (
        r**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # Prevent tiny floating-point errors
    cos_theta2 = max(-1, min(1, cos_theta2))

    # Two possible elbow configurations
    if elbow_up:
        theta2 = math.acos(cos_theta2)
    else:
        theta2 = -math.acos(cos_theta2)

    # Calculate theta1
    theta1 = math.atan2(y, x) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2)
    )

    return theta1, theta2


# Initial target
target_x = 12
target_y = 5

elbow_up = True

solution = inverse_kinematics(
    target_x,
    target_y,
    elbow_up
)

if solution is not None:

    theta1, theta2 = solution

    x1, y1, x2, y2 = get_arm_positions(
        theta1,
        theta2
    )

else:

    theta1 = 0
    theta2 = 0

    x1, y1, x2, y2 = get_arm_positions(
        theta1,
        theta2
    )


# Plot
fig, ax = plt.subplots()

plt.subplots_adjust(
    bottom=0.30,
    right=0.75
)

# Robot arm
line, = ax.plot(
    [0, x1, x2],
    [0, y1, y2],
    'o-',
    linewidth=3
)

# Target
target_point, = ax.plot(
    target_x,
    target_y,
    'rx',
    markersize=12,
    markeredgewidth=3
)

# Workspace
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)

ax.set_aspect('equal')

ax.grid(True)

ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.set_title("2-DOF Robotic Arm")



# Target X slider
ax_x = plt.axes([
    0.20,
    0.15,
    0.50,
    0.03
])

slider_x = Slider(
    ax_x,
    'Target X',
    -18,
    18,
    valinit=target_x
)


# Target Y slider
ax_y = plt.axes([
    0.20,
    0.10,
    0.50,
    0.03
])

slider_y = Slider(
    ax_y,
    'Target Y',
    -18,
    18,
    valinit=target_y
)


# Elbow configuration
ax_radio = plt.axes([
    0.78,
    0.45,
    0.18,
    0.15
])

radio = RadioButtons(
    ax_radio,
    ('Elbow Up', 'Elbow Down')
)


# Text displaying angles
angle_text = ax.text(
    0.02,
    0.95,
    '',
    transform=ax.transAxes,
    verticalalignment='top'
)


# Update function
def update(val):

    global elbow_up

    target_x = slider_x.val
    target_y = slider_y.val

    solution = inverse_kinematics(
        target_x,
        target_y,
        elbow_up
    )

    # Move target
    target_point.set_data(
        [target_x],
        [target_y]
    )

    # If target cannot be reached
    if solution is None:

        angle_text.set_text(
            "Target unreachable!"
        )

        fig.canvas.draw_idle()

        return

    # Get joint angles
    theta1, theta2 = solution

    # Forward kinematics
    x1, y1, x2, y2 = get_arm_positions(
        theta1,
        theta2
    )

    # Update arm
    line.set_data(
        [0, x1, x2],
        [0, y1, y2]
    )

    # Display angles
    angle_text.set_text(
        f"θ1 = {math.degrees(theta1):.1f}°\n"
        f"θ2 = {math.degrees(theta2):.1f}°"
    )

    fig.canvas.draw_idle()


# Elbow radio button
def change_elbow(label):

    global elbow_up

    if label == 'Elbow Up':
        elbow_up = True
    else:
        elbow_up = False

    update(None)


radio.on_clicked(change_elbow)


# Connect sliders
slider_x.on_changed(update)
slider_y.on_changed(update)


# Show initial angles
update(None)

plt.show()