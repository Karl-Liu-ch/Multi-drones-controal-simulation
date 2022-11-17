from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation
from UAV_class import drone
from Obstacle_Class import Obstacle

# fig = plt.figure(figsize=(50, 50))
fig = plt.figure(figsize=(10, 10))
obs = []
for i in range(50):
    obs.append(Obstacle(abs(3 + 3 * np.random.randn()), abs(3 + 3 * np.random.randn()), 0.2, 0.2, 3 * abs(np.random.randn())))
X = np.array([ ob.x for ob in obs])
Y = np.array([ ob.y for ob in obs])
height = np.array([ ob.height for ob in obs])
bottom=np.zeros_like(X)
length=np.array([ ob.length for ob in obs])
width=np.array([ ob.width for ob in obs])

ax = fig.add_subplot(projection='3d')

ax.legend()

ax.bar3d(X, Y, bottom, length, width, height, shade=True)

txt = fig.suptitle('')

q = [[0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5],
     [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],[2, 2, 2, 2, 2, 2, 4, 4, 4, 4, 4, 4]]

x=np.array(q[0], dtype=float)
y=np.array(q[1], dtype=float)
z=np.array(q[2], dtype=float)
points, = ax.plot(x, y, z, 'o')
drones = [drone(x[k], y[k], z[k], 0, 1, 0, 0, 0, 0, 0, obs, x[k], y[k] + 10, z[k]) for k in
              range(12)]

u = np.linspace(0, 2*np.pi, 100)
v = np.linspace(0, np.pi, 100)
r = 0.5

ax.set_xlim(0, 60)
ax.set_ylim(0, 60)
ax.set_zlim(0, 60)

x0 = r * np.outer(np.cos(u), np.sin(v)) + 10
y0 = r * np.outer(np.sin(u), np.sin(v)) + 10
z0 = r * np.outer(np.ones(np.size(u)), np.cos(v)) + 50

surface_color = "tab:blue"

def update_points(t, x, y, z, points):
    txt.set_text('num={:d}'.format(t))
    new_x = []
    new_y = []
    new_z = []
    for j in range(len(drones)):
        drone_cur = drones[j]
        drone_cur.calculate_position()
        drone_cur.calculate_velocity()
        drone_cur.update_t()
        drone_cur.detect_obstacles()
        drone_cur.end_direction()
        px, py, pz, vx, vy, vz, ax, ay, az, t, endx, endy, endz = \
            drone_cur.position[0], drone_cur.position[1], drone_cur.position[2], \
            drone_cur.v[0], drone_cur.v[1], drone_cur.v[2], \
            drone_cur.a[0], drone_cur.a[1], drone_cur.a[2], \
            drone_cur.t, \
            drone_cur.end[0], drone_cur.end[1], drone_cur.end[2]
        # drone_cur = drone(px, py, pz, vx, vy, vz, ax, ay, az, t, obs, endx, endy, endz)
        # px, py, pz, vx, vy, vz, t = drone_cur.x, drone_cur.y, drone_cur.z, drone_cur.vx, drone_cur.vy, drone_cur.vz, drone_cur.t
        # drone_cur = drone(px,py,pz,vx,vy,vz,1 * np.random.randn(),1 * np.random.randn(),1 * np.random.randn(),t)
        new_x.append(px)
        new_y.append(py)
        new_z.append(pz)

    new_x, new_y, new_z = np.array(new_x), np.array(new_y), np.array(new_z)
    points.set_data(new_x, new_y)
    points.set_3d_properties(new_z, 'z')
    return points,

ani=animation.FuncAnimation(fig, update_points, frames=99, fargs=(x, y, z, points))

ax.set_xlabel("x [pc]")
ax.set_ylabel("y [pc]")
ax.set_zlabel('z [pc]')

plt.show()
