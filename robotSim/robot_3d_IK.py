import numpy as np
import math
import matplotlib.pyplot as plt

from robot_3d_FWK import forward_kinematics

# Dimensions

L1 = 5
L2 = 8
L3 = 6

# J1 => Bases
# J2 => Shoulder
# J3 => Elbow


def inverse_kinematics(x,y,z):

    # J1 Base Rotatio :only rotates around Z
    theta1 = math.atan2(y,x)

    # Horizontal Distance from Base
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

    theta3 = -math.acos(cos_theta3)

    # J2 angle

    alpha = math.atan2(z_relative, r)
    beta = math.atan2(L3*math.sin(theta3), L2 +L3*math.cos(theta3))
    theta2 = -(alpha + beta)
    return (theta1, theta2, theta3)


# Testing
# Testing
target = np.array([
    8.0,
    5.0,
    10.0
])


theta1, theta2, theta3 = inverse_kinematics(
    target[0],
    target[1],
    target[2]
)


base, joint2, joint3, end_effector = forward_kinematics(
    theta1,
    theta2,
    theta3
)


end_position = end_effector[:3, 0]


print("TARGET")
print(target)

print()

print("IK ANGLES")

print(
    "Theta 1:",
    math.degrees(theta1)
)

print(
    "Theta 2:",
    math.degrees(theta2)
)

print(
    "Theta 3:",
    math.degrees(theta3)
)

print()

print("FK RESULT")

print(
    "X:", end_position[0]
)

print(
    "Y:", end_position[1]
)

print(
    "Z:", end_position[2]
)

print()

print("ERROR")

error = target - end_position

print(
    "X:", error[0]
)

print(
    "Y:", error[1]
)

print(
    "Z:", error[2]
)

print(
    "Distance:", np.linalg.norm(error)
)