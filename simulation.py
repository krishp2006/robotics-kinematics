import math

import matplotlib.pyplot as plt

L1 = 10  
L2 = 8

def get_arm_positions(theta1, theta2):
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return x1, y1, x2, y2



theta1 = math.radians(50)
theta2 = math.radians(20)

x1, y1, x2, y2 = get_arm_positions(theta1, theta2)

x = [0,x1,x2]
y = [0,y1,y2]

#Plotting 

print("End effector:")
print("x =", x2)
print("y =", y2)

fig, ax = plt.subplots()
ax.plot(x, y)

plt.axis('equal')
plt.show()
