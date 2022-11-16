from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation

def visualization(robots, obstacles):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(projection='3d')
    ax.legend()
    x = []
    y = []
    z = []
    for i in range(len(robots)):
        x.append(robots[i][0, 0])
        y.append(robots[i][1, 0])
        z.append(robots[i][2, 0])

    X = np.array([ob.position[0] for ob in obstacles])
    Y = np.array([ob.position[1] for ob in obstacles])
    height = np.array([ob.position[2] for ob in obstacles])
    bottom = np.zeros_like(X)
    length = np.array([0.5 for ob in obstacles])
    width = np.array([0.5 for ob in obstacles])
    ax.bar3d(X, Y, bottom, length, width, height, shade=True)

    # for j in range(np.shape(obstacles)[2]):
    #     x.append(obstacles[0, 0, j])
    #     y.append(obstacles[1, 0, j])
    #     z.append(obstacles[2, 0, j])
    x = np.array(x, dtype=float)
    y = np.array(y, dtype=float)
    z = np.array(z, dtype=float)
    points, = ax.plot(x, y, z, 'o')
    ani = animation.FuncAnimation(fig, update_points, frames=200, fargs=(robots, points))
    ax.set_xlabel("x [pc]")
    ax.set_ylabel("y [pc]")
    ax.set_zlabel('z [pc]')

    plt.show()

def update_points(t, robots, points):
    new_x = []
    new_y = []
    new_z = []
    for j in range(len(robots)):
        robot = robots[j]
        position = robot[:3, t]
        px, py, pz = position[0], position[1], position[2]
        new_x.append(px)
        new_y.append(py)
        new_z.append(pz)

    # for j in range(np.shape(obstacles)[2]):
    #     new_x.append(obstacles[0, t, j])
    #     new_y.append(obstacles[1, t, j])
    #     new_z.append(obstacles[2, t, j])

    new_x, new_y, new_z = np.array(new_x), np.array(new_y), np.array(new_z)
    points.set_data(new_x, new_y)
    points.set_3d_properties(new_z, 'z')
    return points,