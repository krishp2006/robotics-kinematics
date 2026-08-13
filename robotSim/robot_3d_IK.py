import numpy as np
import math
import matplotlib.pyplot as plt

from matplotlib.widgets import Slider, Button, RadioButtons
from robot_3d_FWK import forward_kinematics

# Dimensions

L1 = 5
L2 = 8
L3 = 6

# J1 => Bases
# J2 => Shoulder
# J3 => Elbow

# Limits

joint_limits = {
    # Base
    "theta1" : ( math.radians(-180), math.radians(180)),

    # Shoulder
    "theta2" : (math.radians(-90), math.radians(90)),

    # Elbow
    "theta3" : (math.radians(-135),math.radians(135))
}

def inverse_kinematics(x,y,z):

    # J1 Base Rotation :only rotates around Z
    theta1 = math.atan2(y,x) # direction from base to target in XY plane

    # Horizontal Distance from Base (z axis)
    r = math.sqrt(x**2 + y**2)

    z_relative = z - L1

    # Distance from J2 to Target
    d = math.sqrt(r**2 + z_relative**2)

    # Error checking for reach limit
    if d > L2 + L3:
        raise ValueError("Target is too far away")

    if d < abs(L2 - L3):
        raise ValueError("Target is too close to the robot")


    # To find J3 angle 
    cos_theta3 = ((d**2 -L2**2 - L3**2) / (2 * L2 * L3))

    cos_theta3 = np.clip(cos_theta3, -1, 1) #Make sure no FP errors     

    theta3_magnitude = math.acos(cos_theta3)

    # Elbow config possibilities
    theta3_options = [-theta3_magnitude, theta3_magnitude]

    solutions = []

    for theta3 in theta3_options:

        alpha = math.atan2(z_relative,r)
        beta = math.atan2(L3*math.sin(theta3), L2 +L3*math.cos(theta3))

        theta2 = -(alpha + beta)
        solutions.append((theta1,theta2,theta3))

    return solutions

# Joint limit checks

def check_joint_limits(theta1, theta2, theta3): 
    if not(joint_limits["theta1"][0] <= theta1 <= joint_limits["theta1"][1]):
        return False
    if not(joint_limits["theta2"][0] <= theta2 <= joint_limits["theta2"][1]):
            return False
    if not(joint_limits["theta3"][0] <= theta3 <= joint_limits["theta3"][1]):
            return False
    return True


# Initial State
current_angles = np.array([math.radians(45), math.radians(30),math.radians(-20)])

# Initial Target
target = np.array([8.0, 5.0, 10.0])

# Getting robots positions
def get_positions(base, joint2, joint3, end_effector):
     x = [base[0,0], joint2[0,0] ,joint3[0,0], end_effector[0,0]]
     y = [base[1,0], joint2[1,0] ,joint3[1,0], end_effector[1,0]]
     z = [base[2,0], joint2[2,0] ,joint3[2,0], end_effector[2,0]]

     return x, y, z

# FWK current state
base, joint2, joint3, end_effector = forward_kinematics(current_angles[0], current_angles[1], current_angles[2])

x, y ,z = get_positions(base, joint2, joint3, end_effector)


# Figure creation

fig = plt.figure(figsize=(16,8))

ax = fig.add_axes(
     [0.05,0.15,0.58,0.75],
     projection= "3d"
)

# Robot lines
line, = ax.plot(x,y,z,"o-", linewidth = 3)

# Target point

target_point, = ax.plot(
     [target[0]],[target[1]],[target[2]], 
     "x",
     markersize = 12,
     markeredgewidth = 3
)

# Plot settings

ax.set_xlim(-20,20)
ax.set_ylim(-20,20)
ax.set_zlim(-20,20)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_title( "3-DOF Robot - IK Sim")

ax.set_box_aspect([1,1,1])

ax.grid(True)

# Debug info
debug_text = fig.text(
     0.02,0.85,
     "",
     fontsize=10,
     family = "monospace",
     verticalalignment = "top"
)

# Target X slider
ax_target_x = fig.add_axes(
    [
        0.75,
        0.42,
        0.20,
        0.03
    ]
)


slider_target_x = Slider(
    ax_target_x,
    "Target X",
    -20,
    20,
    valinit=8
)

# Target Y slider
ax_target_y = fig.add_axes(
    [
        0.75,
        0.37,
        0.20,
        0.03
    ]
)


slider_target_y = Slider(
    ax_target_y,
    "Target Y",
    -20,
    20,
    valinit=5
)

# Target Z slider
ax_target_z = fig.add_axes(
    [
        0.75,
        0.32,
        0.20,
        0.03
    ]
)


slider_target_z = Slider(
    ax_target_z,
    "Target Z",
    0,
    20,
    valinit=10
)

# Elbow selection
ax_solution = fig.add_axes([0.76,0.18,0.18,0.10])

solution_selector = RadioButtons(ax_solution, ["Elbow 1", "Elbow 2"])

# Solve button 

ax_solve = fig.add_axes(
     [0.76,0.10,0.18,0.05]
)

solve_button = Button(ax_solve, "Solve IK")

# Update target visual
def update_target():

    target[0] = slider_target_x.val
    target[1] = slider_target_y.val
    target[2] = slider_target_z.val

    target_point.set_data_3d(
        [target[0]],
        [target[1]],
        [target[2]]
    )

def move_robot(target_angles):

    global current_angles

    # Starting joint configuration

    start_angles = current_angles.copy()

    # Number of movement steps

    steps = 50

    for i in range(steps + 1):

        # 0 to 1

        t = i / steps

        # Interpolate between current and target angles

        angles = (
            start_angles
            + t * (
                np.array(target_angles)
                - start_angles
            )
        )

        theta1 = angles[0]
        theta2 = angles[1]
        theta3 = angles[2]

        # Forward kinematics

        base, joint2, joint3, end_effector = forward_kinematics(
            theta1,
            theta2,
            theta3
        )

        # Get robot positions

        x, y, z = get_positions(
            base,
            joint2,
            joint3,
            end_effector
        )

        # Update robot line

        line.set_data_3d(
            x,
            y,
            z
        )

        # Update screen

        fig.canvas.draw_idle()

        plt.pause(0.01)

    # Save final joint configuration

    current_angles = np.array(
        target_angles
    )


# Solve IK

def solve_target(event):

    update_target()

    # Try to solve IK

    try:

        solutions = inverse_kinematics(
            target[0],
            target[1],
            target[2]
        )

    except ValueError as error:

        debug_text.set_text(
            f"""
IK STATUS

ERROR

{error}
"""
        )

        fig.canvas.draw_idle()

        return


    # Select elbow configuration

    if solution_selector.value_selected == "Elbow 1":

        solution_index = 0

    else:

        solution_index = 1


    selected_solution = solutions[solution_index]

    theta1, theta2, theta3 = selected_solution


    # Check joint limits

    if not check_joint_limits(
        theta1,
        theta2,
        theta3
    ):

        debug_text.set_text(
            """
IK STATUS

TARGET IS REACHABLE

BUT THE SELECTED
SOLUTION VIOLATES
JOINT LIMITS.

Try the other
elbow configuration.
"""
        )

        fig.canvas.draw_idle()

        return


    # Forward kinematics verification

    base, joint2, joint3, end_effector = forward_kinematics(
        theta1,
        theta2,
        theta3
    )


    # Get actual end effector position

    end_position = end_effector[:3, 0]


    # Calculate error

    error_vector = target - end_position

    error = np.linalg.norm(
        error_vector
    )


    # Move robot

    move_robot(
        selected_solution
    )


    # Display debug information

    debug_text.set_text(
        f"""
IK SOLUTION

Configuration:
{solution_selector.value_selected}


JOINT ANGLES

J1 Base:     {math.degrees(theta1):8.2f}°
J2 Shoulder: {math.degrees(theta2):8.2f}°
J3 Elbow:    {math.degrees(theta3):8.2f}°


TARGET

X: {target[0]:8.2f}
Y: {target[1]:8.2f}
Z: {target[2]:8.2f}


END EFFECTOR

X: {end_position[0]:8.2f}
Y: {end_position[1]:8.2f}
Z: {end_position[2]:8.2f}


ERROR

X: {error_vector[0]:8.5f}
Y: {error_vector[1]:8.5f}
Z: {error_vector[2]:8.5f}

Distance: {error:8.5f}
"""
    )

    fig.canvas.draw_idle()


# Reset view button

ax_reset = fig.add_axes(
    [
        0.76,
        0.03,
        0.18,
        0.05
    ]
)

reset_button = Button(
    ax_reset,
    "Reset View"
)


# Reset view

def reset_view(event):

    ax.view_init(
        elev=25,
        azim=-60
    )

    fig.canvas.draw_idle()


# Connect buttons

reset_button.on_clicked(
    reset_view
)

solve_button.on_clicked(
    solve_target
)


# Initial display

update_target()

debug_text.set_text(
    """
3-DOF ROBOT IK

Move the XYZ sliders.

Select an elbow
configuration.

Press SOLVE IK.
"""
)


# Run

if __name__ == "__main__":

    plt.show()