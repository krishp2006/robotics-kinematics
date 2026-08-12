import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

L1 = 10
L2 = 8


def get_arm_positions(theta1, theta2):
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return x1, y1, x2, y2



def inverse_kinematics(x,y):
    
    r = math.sqrt(x**2 + y**2)

    cos_theta2 = (x**2 + y**2 -L1**2 - L2**2) / (2 * L1 * L2)

    theta2 = math.acos(cos_theta2)  

    theta1 = math.atan2(y, x) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2)
    )
    return theta1, theta2

theta1, theta2 = inverse_kinematics(10, 10)


x1, y1, x2, y2 = get_arm_positions(theta1, theta2)

print("Target:")
print("x =", )
print("y =", 5)

print("\nFK result:")
print("x =", x2)
print("y =", y2)

# Initial angles
theta1 = math.radians(0)
theta2 = math.radians(0)

x1, y1, x2, y2 = get_arm_positions(theta1, theta2)

x = [0, x1, x2]
y = [0, y1, y2]


# Plotting
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)

# Create the arm
line, = ax.plot(x, y, 'o-')

ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_aspect('equal')


# Create slider areas
ax_theta1 = plt.axes([0.2, 0.1, 0.65, 0.03])
ax_theta2 = plt.axes([0.2, 0.05, 0.65, 0.03])

slider_theta1 = Slider(
    ax_theta1,
    'Theta 1',
    0,
    180,
    valinit=0
)

slider_theta2 = Slider(
    ax_theta2,
    'Theta 2',
    -135,
    135,
    valinit=0
)

# Update the arm when a slider moves
def update(val):

    theta1 = math.radians(slider_theta1.val)
    theta2 = math.radians(slider_theta2.val)

    x1, y1, x2, y2 = get_arm_positions(theta1, theta2)

    x = [0, x1, x2]
    y = [0, y1, y2]

    line.set_data(x, y)

    fig.canvas.draw_idle()


slider_theta1.on_changed(update)
slider_theta2.on_changed(update)


plt.show()