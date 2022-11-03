from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation
from UAV_class import px, py, pz, pt

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# create the parametric curve
# t=np.arange(0, 2*np.pi, 2*np.pi/100)
# x=np.cos(t)
# y=np.sin(t)
# z=t/(2.*np.pi)

t = pt
x = px
y = py
z = pz

# create the first plot
point, = ax.plot([x[0]], [y[0]], [z[0]], 'o')
line, = ax.plot(x, y, z, label='parametric curve')
ax.legend()
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])

# second option - move the point position at every frame
def update_point(n, x, y, z, point):
    point.set_data(x[n:n+1], y[n:n+1])
    point.set_3d_properties(z[n:n+1], 'z')
    return point

ani=animation.FuncAnimation(fig, update_point, 99, fargs=(x, y, z, point))

plt.show()