import numpy as np
import math
import matplotlib.pyplot as plt
import pygame


from robot_3d_FWK import forward_kinematics
from robot_3d_IK import inverse_kinematics, check_joint_limits


# Dimensions

L1 = 5
L2 = 8
L3 = 6


# Initial robot angles

current_angles = np.array([
    math.radians(45),
    math.radians(30),
    math.radians(-20)
])


# Initial target

target = np.array([
    8.0,
    5.0,
    10.0
])


# Elbow configuration

elbow_configuration = 0


# Controller deadzone

DEADZONE = 0.15


# Target movement speed

TARGET_SPEED = 5.0


# Get robot positions
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



# controller
pygame.init()

pygame.joystick.init()


if pygame.joystick.get_count() == 0:

    print("No controller detected.")

    pygame.quit()

    raise SystemExit


controller = pygame.joystick.Joystick(0)

controller.init()


print("Controller connected:")
print(controller.get_name())



# Initial position
base, joint2, joint3, end_effector = forward_kinematics(
    current_angles[0],
    current_angles[1],
    current_angles[2]
)


x, y, z = get_positions(
    base,
    joint2,
    joint3,
    end_effector
)


# Figure
plt.ion()

fig = plt.figure(
    figsize=(16, 8)
)


ax = fig.add_axes(
    [0.05, 0.10, 0.58, 0.80],
    projection="3d"
)


# Robot
line, = ax.plot(
    x,
    y,
    z,
    "o-",
    linewidth=3
)


# Target

target_point, = ax.plot(
    [target[0]],
    [target[1]],
    [target[2]],
    "x",
    markersize=12,
    markeredgewidth=3
)


# Plot settings
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-20, 20)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_title(
    "3-DOF Robot - Xbox Controller"
)

ax.set_box_aspect(
    [1, 1, 1]
)

ax.grid(True)


# Debug text
debug_text = fig.text(
    0.67,
    0.80,
    "",
    fontsize=10,
    family="monospace",
    verticalalignment="top"
)


# Deadzone
def apply_deadzone(value):

    if abs(value) < DEADZONE:

        return 0

    return value

plt.show(block=False)


# Main controller loop
running = True

clock = pygame.time.Clock()


while running:

    # controller events
    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            running = False


        # A button
        if event.type == pygame.JOYBUTTONDOWN:

            if event.button == 0:

                elbow_configuration += 1

                elbow_configuration %= 2

                print(
                    "Elbow configuration:",
                    elbow_configuration + 1
                )


            # B button
            if event.button == 1:

                running = False


    # Read sticks for input
    left_x = controller.get_axis(0)

    left_y = controller.get_axis(1)

    right_y = controller.get_axis(3)


    # Apply deadzone

    left_x = apply_deadzone(left_x)

    left_y = apply_deadzone(left_y)

    right_y = apply_deadzone(right_y)


    # Move targets
    dt = clock.get_time() / 1000.0


    target[0] += left_x * TARGET_SPEED * dt

    target[1] -= left_y * TARGET_SPEED * dt

    target[2] -= right_y * TARGET_SPEED * dt


    # Limiting target area
    target[0] = np.clip(
        target[0],
        -20,
        20
    )

    target[1] = np.clip(
        target[1],
        -20,
        20
    )

    target[2] = np.clip(
        target[2],
        0,
        20
    )


    # Updating target point
    target_point.set_data_3d(
        [target[0]],
        [target[1]],
        [target[2]]
    )


    # Solving IK
    try:

        solutions = inverse_kinematics(
            target[0],
            target[1],
            target[2]
        )


        selected_solution = solutions[
            elbow_configuration
        ]


        theta1, theta2, theta3 = selected_solution


        # Check joint limits
        valid = check_joint_limits(
            theta1,
            theta2,
            theta3
        )


        if valid:

            target_angles = np.array(
                selected_solution
            )

            # Smooth robot movements
            movement_speed = 8.0


            difference = (
                target_angles
                - current_angles
            )


            max_change = (
                movement_speed * dt
            )


            for i in range(3):

                if abs(difference[i]) > max_change:

                    current_angles[i] += (
                        math.copysign(
                            max_change,
                            difference[i]
                        )
                    )

                else:

                    current_angles[i] = (
                        target_angles[i]
                    )


            # Forward kinematics
            base, joint2, joint3, end_effector = forward_kinematics(
                current_angles[0],
                current_angles[1],
                current_angles[2]
            )

            # Get positions
            x, y, z = get_positions(
                base,
                joint2,
                joint3,
                end_effector
            )


            # Update robot
            line.set_data_3d(
                x,
                y,
                z
            )

            # End effector
            end_position = (
                end_effector[:3, 0]
            )


            error_vector = (
                target
                - end_position
            )


            error = np.linalg.norm(
                error_vector
            )


            # debug
            debug_text.set_text(
                f"""
XBOX ROBOT CONTROL


TARGET

X: {target[0]:8.2f}
Y: {target[1]:8.2f}
Z: {target[2]:8.2f}


JOINT ANGLES

J1 Base:     {math.degrees(current_angles[0]):8.2f}°
J2 Shoulder: {math.degrees(current_angles[1]):8.2f}°
J3 Elbow:    {math.degrees(current_angles[2]):8.2f}°


TARGET JOINT ANGLES

J1: {math.degrees(theta1):8.2f}°
J2: {math.degrees(theta2):8.2f}°
J3: {math.degrees(theta3):8.2f}°


END EFFECTOR

X: {end_position[0]:8.2f}
Y: {end_position[1]:8.2f}
Z: {end_position[2]:8.2f}


ERROR

X: {error_vector[0]:8.3f}
Y: {error_vector[1]:8.3f}
Z: {error_vector[2]:8.3f}

Distance: {error:8.3f}


ELBOW CONFIGURATION

{elbow_configuration + 1}
"""
            )


        else:

            debug_text.set_text(
                f"""
XBOX ROBOT CONTROL


TARGET

X: {target[0]:8.2f}
Y: {target[1]:8.2f}
Z: {target[2]:8.2f}


STATUS

TARGET REACHABLE

BUT JOINT LIMITS
ARE VIOLATED.


Press A to switch
elbow configuration.
"""
            )


    except ValueError:

        debug_text.set_text(
            f"""
XBOX ROBOT CONTROL


TARGET

X: {target[0]:8.2f}
Y: {target[1]:8.2f}
Z: {target[2]:8.2f}


STATUS

TARGET OUT OF
ROBOT REACH.
"""
        )


    # Updating screen
    fig.canvas.draw_idle()

    fig.canvas.flush_events()

    # Control loop speed
    clock.tick(60)


# cleanup
pygame.quit()

plt.close(fig)