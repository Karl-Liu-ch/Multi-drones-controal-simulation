from matplotlib import pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from UAV_class import drone

fig = plt.figure()
ax = p3.Axes3D(fig)

q = [[0, 1, 2, 3, 4, 5],
     [ 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]]
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

points, = ax.plot(x, y, z, 'o')
txt = fig.suptitle('')

drones = [drone(x[k], y[k], z[k], 0, 0, 0, 1 * np.random.randn(), 1 * np.random.randn(), 1 * np.random.randn(), 0) for k in
              range(6)]

def update_points(t, x, y, z, points):
    txt.set_text('num={:d}'.format(t))
    new_x = []
    new_y = []
    new_z = []
    for j in range(len(drones)):
        drone_cur = drones[j]
        drone_cur.calculate_velocity(1 * np.random.randn(),1 * np.random.randn(),1 * np.random.randn())
        drone_cur.calculate_position()
        drone_cur.update_t()
        px, py, pz, vx, vy, vz, t = drone_cur.x, drone_cur.y, drone_cur.z, drone_cur.vx, drone_cur.vy, drone_cur.vz, drone_cur.t
        drone_cur = drone(px,py,pz,vx,vy,vz,1 * np.random.randn(),1 * np.random.randn(),1 * np.random.randn(),t)
        new_x.append(px)
        new_y.append(py)
        new_z.append(pz)

    new_x, new_y, new_z = np.array(new_x), np.array(new_y), np.array(new_z)
    points.set_data(new_x, new_y)
    points.set_3d_properties(new_z, 'z')
    return points, txt

ani=animation.FuncAnimation(fig, update_points, frames=99, fargs=(x, y, z, points))

ax.set_xlabel("x [pc]")
ax.set_ylabel("y [pc]")
ax.set_zlabel('z [pc]')
plt.show()