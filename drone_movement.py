from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation
from UAV_class import px, py, pz, pt

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

points = [[] for k in range(len(pt))] 
lines = [[] for k in range(len(pt))]
properties = [{
    't' : [],
    'x' : [],
    'y' : [],
    'z' : []
} for k in range(len(pt))]

for i in range(len(pt)):
    properties[i]['t'] = pt[i]
    properties[i]['x'] = px[i]
    properties[i]['y'] = py[i]
    properties[i]['z'] = pz[i]
    points[i], = ax.plot([properties[i]['x'][0]], [properties[i]['y'][0]], [properties[i]['z'][0]], 'o')
    print([properties[i]['x'][0]])
    lines[i], = ax.plot(properties[i]['x'], properties[i]['y'], properties[i]['z'])

def update_points(n, properties, points):
    for i in range(len(points)):
        points[i].set_data(properties[i]['x'][n:n+1], properties[i]['y'][n:n+1])
        points[i].set_3d_properties(properties[i]['z'][n:n+1], 'z')
    return points

#ani=animation.FuncAnimation(fig, update_point, 99, fargs=(x, y, z, point))
ani=animation.FuncAnimation(fig, update_points, 99, fargs=(properties, points))

plt.show()