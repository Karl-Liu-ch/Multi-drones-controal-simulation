from matplotlib import pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation


fig = plt.figure()
ax = p3.Axes3D(fig)

q = [[-4.32, -2.17, -2.25, 4.72, 2.97, 1.74],
     [ 2.45, 9.73,  7.45,4.01,3.42,  1.80],[-1.40, -1.76, -3.08,-9.94,-3.13,-1.13]]
v = [[ 0.0068,0.024, -0.014,-0.013, -0.0068,-0.04],[ 0.012,
      0.056, -0.022,0.016,  0.0045, 0.039],
     [-0.0045,  0.031,  0.077,0.0016, -0.015,-0.00012]]

x=np.array(q[0])
y=np.array(q[1])
z=np.array(q[2])
s=np.array(v[0])
u=np.array(v[1])
w=np.array(v[2])

ax.legend()
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])

points, = ax.plot(x, y, z, '*')
txt = fig.suptitle('')

def update_points(t, x, y, z, points):
    # txt.set_text('num={:d}'.format(t))

    new_x = x + s * t
    new_y = y + u * t
    new_z = z + w * t
    # print('t:', t)

    # update properties
    points.set_data([x[0],new_x[0]],[y[0],new_y[0]])
    points.set_3d_properties([z[0],new_z[0]], 'z')

    # return modified artists
    return points

ani=animation.FuncAnimation(fig, update_points, frames=99, fargs=(x, y, z, points))

# ax.set_xlabel("x [pc]")
# ax.set_ylabel("y [pc]")
# ax.set_zlabel('z [pc]')
plt.show()