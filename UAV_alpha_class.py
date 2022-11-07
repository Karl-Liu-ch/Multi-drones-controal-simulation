import numpy as np
from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation
from Obstacle_Class import Obstacle
import random

class Environment():
    def __init__(self, num_obs, num_drones):
        self.obs = [Obstacle(3 + 3 * np.random.randn(), 3 + 3 * np.random.randn(), 0.2, 0.2, 3 * abs(np.random.randn())) for i in range(num_obs)]
        self.drones = [drone() for k in range(num_drones)]
        self.obstate = self.get_obs_state()
        self.start_point = self.set_start()
    
    def calDistance(self, x, y):
        return np.linalg.norm(x - y)
    
    def collsion(self):
        reward = 0
        for drone in self.drones:
            for ob in self.obs:
                d = self.calDistance(drone.position[0:2], np.array([ob.x + ob.length * 0.5, ob.y + ob.width * 0.5]))
                d_threshold = 0.2 + ob.length
                if d < d_threshold:
                    # print("crash")
                    reward = reward - 10 * d_threshold / d
                    return reward
        for i in range(len(self.drones)):
            for j in range(len(self.drones)):
                if i != j:
                    d = self.calDistance(self.drones[i].position, self.drones[j].position)
                    d_threshold = 0.4
                    if d < d_threshold:
                        # print("crash")
                        reward = reward - 10 * d_threshold / d
                    return reward
        return 0
    
    def get_obs_state(self):
        state = []
        for ob in self.obs:
            state.append(ob.x)
            state.append(ob.y)
            state.append(ob.height)
        return np.array(state)
    
    def set_start(self):
        k = 0.0
        start_point = np.array([])
        for drone in self.drones:
            drone.set_start(np.array([k, 0.0, 0.0]))
            start_point = np.append(start_point, np.array([k, 0.0, 0.0]))
            k = k + 1.0
        return start_point

    def set_end(self):
        k = 0.0
        for drone in self.drones:
            drone.set_end(np.array([k + np.random.randn(), 10.0 + np.random.randn(), 10.0 + np.random.randn()]))
            k = k + 1.0
    
    def set_va(self):
        for drone in self.drones:
            drone.set_va()
        
    def get_state(self):
        state = []
        for drone in self.drones:
            state.append(drone.get_state())
        return np.array(state).flatten()
    
    def reset(self):
        state = []
        for drone in self.drones:
            drone.reset()
        self.set_start()
        self.set_end()
        state = self.get_state()
        state = np.append(state, self.obstate)
        return state
    
    def step(self, action):
        states = []
        dones = []
        reward = 0
        for i in range(len(self.drones)):
            a = action[i]
            state, reward, done = self.drones[i].step(a)
            reward = reward + self.collsion() * 1e2
            # reward = reward - d + 10.0 / (d + 1.0) - self.drones[i].t + 10.0 / (self.drones[i].t + 1.0)
            states.append(self.drones[i].get_state())
            dones.append(done)
        states = np.array(states).flatten()
        states = np.append(states, self.obstate)
        if all(dones) == True:
            Done = True
        else:
            Done = False
        return states, reward, Done
    
class drone():
    def __init__(self):
        self.position = np.array([0,0,0])
        self.start_point = np.array([0,0,0])
        self.V = 1.0
        self.alpha = 0.5 * np.pi
        self.phi = 0
        v = np.array([self.V * np.cos(self.phi) * np.cos(self.alpha), self.V * np.cos(self.phi) * np.sin(self.alpha), self.V * np.sin(self.phi)])
        self.v = v / np.linalg.norm(v) * self.V
        self.xmax = 10 / 180 * np.pi  # 偏航角速度最大值  每个步长允许变化的角度
        self.gammax = 10 / 180 * np.pi  # 爬升角速度最大值  每个步长允许变化的角度
        
        self.t = 0
        self.dt = 1e-1
        self.end = np.array([10 * np.random.randn(),10 * np.random.randn(),abs(10 * np.random.randn())])
        self.avoiding = False
    
    def set_end(self, end):
        self.end = end
        
    def set_start(self, start):
        self.position = start
        self.start_point = start
    
    def set_va(self):
        self.v = np.array([self.V * np.cos(self.phi) * np.cos(self.alpha), self.V * np.cos(self.phi) * np.sin(self.alpha), self.V * np.sin(self.phi)])
        self.v = self.v / np.linalg.norm(self.v) * self.V

    def get_state(self):
        state = np.append(self.position, np.append(self.v, np.append(self.alpha, np.append(self.phi, self.end))))
        state = np.insert(state, 11, self.t)
        return state
        
    def update_state(self, d_alpha, d_phi):
        self.alpha = self.alpha + d_alpha
        while self.alpha > np.pi:
            self.alpha = self.alpha - np.pi
        while self.alpha < - np.pi:
            self.alpha = self.alpha + np.pi
        self.phi = self.phi + d_phi
        while self.phi > np.pi:
            self.phi = self.phi - np.pi
        while self.phi < - np.pi:
            self.phi = self.phi + np.pi
        self.v = np.array([self.V * np.cos(self.phi) * np.cos(self.alpha), self.V * np.cos(self.phi) * np.sin(self.alpha), self.V * np.sin(self.phi)])
        self.v = self.v / np.linalg.norm(self.v) * self.V
        self.position = self.position + self.v * self.dt
        self.t = self.t + self.dt
    
    def step(self, action):
        done, d = self.end_direction()
        if done:
            self.v = np.array([0,0,0])
            self.V = 0
            d_start = self.distance(self.position, self.start_point)
            reward = - d / (d_start + 1.0) - self.t / 10.0 
            state = np.append(self.position, np.append(self.v, np.append(self.alpha, np.append(self.phi, self.end))))
            state = np.insert(state, 11, self.t)
            return state, reward, done
        else:
            self.update_state(action[0], action[1])
            d_start = self.distance(self.position, self.start_point)
            reward = - d / (d_start + 1.0) - self.t / 10.0 
            state = np.append(self.position, np.append(self.v, np.append(self.alpha, np.append(self.phi, self.end))))
            state = np.insert(state, 11, self.t)
        return state, reward, done
        
    def end_direction(self):
        d = self.end - self.position
        # if np.linalg.norm(d) < 1e-2:
        if np.linalg.norm(d) < 1e-2 or self.t > 10:
            return True, np.linalg.norm(d)
        else:
            return False, np.linalg.norm(d)
    
    def distance(self, a, b):
        return np.linalg.norm(a - b)
    
    def reset(self):
        self.t = 0.0
        self.position = np.array([0,0,0])
        self.V = 1.0
        self.alpha = 0.0
        self.phi = 0.0
        v = np.array([self.V * np.cos(self.phi) * np.cos(self.alpha), self.V * np.cos(self.phi) * np.sin(self.alpha), self.V * np.sin(self.phi)])
        self.v = v / np.linalg.norm(v) * self.V
        state = np.append(self.position, np.append(self.v, np.append(self.alpha, np.append(self.phi, self.end))))
        state = np.insert(state, 11, self.t)
        return state

if __name__ == "__main__":
    # define random obstacles
    obs = []
    for i in range(30):
        obs.append(Obstacle(3 + 3 * np.random.randn(), 3 + 3 * np.random.randn(), 0.3, 0.3, 4 * abs(np.random.randn())))

    q = [[0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [2, 2, 2, 2, 2, 2, 4, 4, 4, 4, 4, 4]]

    x = np.array(2 * q[0], dtype=float)
    y = np.array(q[1] - np.array([5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]), dtype=float)
    z = np.array(q[2], dtype=float)
    drones = [drone() for k in range(12)]
    for k in range(len(drones)):
        drone_cur = drones[k]
        drone_cur.set_start(np.array([x[k], y[k], z[k]]))
    px = [[] for k in range(len(drones))]
    py = [[] for k in range(len(drones))]
    pz = [[] for k in range(len(drones))]
    pt = [[] for k in range(len(drones))]

    for j in range(len(drones)):
        drone_cur = drones[j]
        for i in range(100):
            drone_cur.update_state(np.random.randn() * 2 / 180 * np.pi, np.random.randn() * 2 / 180 * np.pi)
            [x, y, z], t= drone_cur.position, drone_cur.t
            px[j].append(x)
            py[j].append(y)
            pz[j].append(z)
            pt[j].append(t)

    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(projection='3d')
    X = np.array([ob.x for ob in obs])
    Y = np.array([ob.y for ob in obs])
    height = np.array([ob.height for ob in obs])
    bottom = np.zeros_like(X)  # 设置柱状图的底端位值
    height = height
    length = np.array([ob.length for ob in obs])
    width = np.array([ob.width for ob in obs])  # 每一个柱子的长和宽
    # 绘图设置
    ax.bar3d(X, Y, bottom, length, width, height, shade=True)  #

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
        lines[i], = ax.plot(properties[i]['x'], properties[i]['y'], properties[i]['z'])

    def update_points(n, properties, points):
        for i in range(len(points)):
            points[i].set_data(properties[i]['x'][n:n+1], properties[i]['y'][n:n+1])
            points[i].set_3d_properties(properties[i]['z'][n:n+1], 'z')
        return points

    ani=animation.FuncAnimation(fig, update_points, 1000, fargs=(properties, points))

    plt.show()