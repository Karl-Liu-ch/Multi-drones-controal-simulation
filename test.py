from matplotlib import pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from UAV_class import drone
from Obstacle_Class import Obstacle
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(50, 50))
X=np.arange(0, 5, step=1)#X轴的坐标
Y=np.arange(0, 9, step=1)#Y轴的坐标
Z=np.zeros(shape=(5, 9))
for i in range(5):
    for j in range(9):
        Z[i, j]=i+j
xx, yy=np.meshgrid(X, Y)#网格化坐标
X, Y=xx.ravel(), yy.ravel()#矩阵扁平化
bottom=np.zeros_like(X)#设置柱状图的底端位值
Z=Z.ravel()#扁平化矩阵
width=height=0.1#每一个柱子的长和宽

ax = p3.Axes3D(fig)

q = [[0, 1, 2, 3, 4, 5],
     [ 0, 0, 0, 0, 0, 0],[2, 2, 2, 2, 2, 2]]

x=np.array(q[0])
y=np.array(q[1])
z=np.array(q[2])

ax.legend()

points, = ax.plot(x, y, z, 'o')
ax.bar3d(X, Y, bottom, width, height, Z, shade=True)

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
        if drone_cur.vy < 5:
            drone_cur.calculate_velocity(0, 10, 0)
        else:
            drone_cur.calculate_velocity(0, 0, 0)
        drone_cur.calculate_position()
        drone_cur.update_t()
        px, py, pz, vx, vy, vz, t = drone_cur.x, drone_cur.y, drone_cur.z, drone_cur.vx, drone_cur.vy, drone_cur.vz, drone_cur.t
        # drone_cur = drone(px,py,pz,vx,vy,vz,1 * np.random.randn(),1 * np.random.randn(),1 * np.random.randn(),t)
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