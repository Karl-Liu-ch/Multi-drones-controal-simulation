from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation

def visualization(obs, pt, px, py, pz):
    fig = plt.figure(figsize=(10, 10))
    X = np.array([ob.x for ob in obs])
    Y = np.array([ob.y for ob in obs])
    height = np.array([ob.height for ob in obs])
    bottom = np.zeros_like(X)
    length = np.array([ob.length for ob in obs])
    width = np.array([ob.width for ob in obs])
    ax = fig.add_subplot(projection='3d')
    ax.legend()
    ax.bar3d(X, Y, bottom, length, width, height, shade=True)
    points = [[] for k in range(len(pt))]
    lines = [[] for k in range(len(pt))]
    properties = [{
        't': [],
        'x': [],
        'y': [],
        'z': []
    } for k in range(len(pt))]

    for i in range(len(pt)):
        properties[i]['t'] = pt[i]
        properties[i]['x'] = px[i]
        properties[i]['y'] = py[i]
        properties[i]['z'] = pz[i]
        points[i], = ax.plot([properties[i]['x'][0]], [properties[i]['y'][0]], [properties[i]['z'][0]], 'o')
        lines[i], = ax.plot(properties[i]['x'], properties[i]['y'], properties[i]['z'])

    def update_points(n, properties, points):
        for i in range(len(points)):
            points[i].set_data(properties[i]['x'][n:n + 1], properties[i]['y'][n:n + 1])
            points[i].set_3d_properties(properties[i]['z'][n:n + 1], 'z')
        return points

    ani = animation.FuncAnimation(fig, update_points, 99, fargs=(properties, points))
    plt.show()