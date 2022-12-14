from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation
from Obstacle_Class import Obstacle
import random

'''
class drone():
    def __init__(self, x, y, z, vx, vy, vz, ax, ay, az, t, obstacle, end_x, end_y, end_z):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.ax = ax
        self.ay = ay
        self.az = az
        self.t = t
        self.dt = 1e-1
        self.obstacle = obstacle
        self.end_x = end_x
        self.end_y = end_y
        self.end_z = end_z
        self.avoiding = False
        if np.sqrt(float(vx) ** 2.0 + float(vy) ** 2) == 0:
            self.alpha = None
        else:
            self.alpha = np.arcsin((float(vx) / np.sqrt(float(vx) ** 2.0 + float(vy) ** 2)))
        if np.sqrt(float(vx) ** 2.0 + float(vy) ** 2 + float(vz) ** 2.0) == 0:
            self.phi = None
        else:
            self.phi = np.arcsin((float(vz) / np.sqrt(float(vx) ** 2.0 + float(vy) ** 2 + float(vz) ** 2.0)))
    
    def calculate_velocity(self):
        # self.ax, self.ay, self.az = ax, ay, az
        self.vx = self.vx + self.ax * self.dt
        self.vy = self.vy + self.ay * self.dt
        self.vz = self.vz + self.az * self.dt

    def calculate_position(self):
        self.x = self.x + self.vx * self.dt + 0.5 * self.ax * (self.dt ** 2)
        self.y = self.y + self.vy * self.dt + 0.5 * self.ay * (self.dt ** 2)
        self.z = self.z + self.vz * self.dt + 0.5 * self.az * (self.dt ** 2)

    def calculate_direction(self):
        if np.sqrt(float(self.vx) ** 2.0 + float(self.vy) ** 2) == 0:
            self.alpha = None
        else:
            self.alpha = np.arcsin((float(self.vx) / np.sqrt(float(self.vx) ** 2.0 + float(self.vy) ** 2)))
        if np.sqrt(float(self.vx) ** 2.0 + float(self.vy) ** 2 + float(self.vz) ** 2.0) == 0:
            self.phi = None
        else:
            self.phi = np.arcsin((float(self.vz) / np.sqrt(float(self.vx) ** 2.0 + float(self.vy) ** 2 + float(self.vz) ** 2.0)))
        return self.alpha, self.phi

    def update_t(self):
        self.t = self.t + self.dt

    def distance(self, ob):
        return np.sqrt((self.x - ob.x) ** 2 + (self.y - ob.y) ** 2), np.sqrt((self.x - ob.x) ** 2 + (self.y - ob.y) ** 2 + (self.z - ob.height) ** 2)

    def detect_obstacles(self):
        self.avoiding = False
        for ob in self.obstacle:
            alpha, phi = self.calculate_direction()
            alpha_min = np.arcsin((ob.x - self.x)/ np.sqrt((ob.x - self.x) ** 2 + (ob.y + ob.width + 2 - self.y) ** 2))
            alpha_max = np.arcsin((ob.x + ob.length + 2 - self.x)/ np.sqrt((ob.x + ob.length + 2 - self.x) ** 2 + (ob.y - self.y) ** 2))
            xy_dis, z_max = self.distance(ob)
            if (self.x > ob.x) and (self.x < ob.x + ob.length) and (self.y > ob.y) and (self.y < ob.y + ob.width) and (self.z < ob.height):
                print("collision")
            if alpha != None:
                if (alpha < alpha_max) and (alpha > alpha_min) and (xy_dis < 2) and (self.z < ob.height):
                    self.avoiding = True
                    # self.ax = 2 * random.choice([1, -1])
                    # self.calculate_velocity()
                    # print("collision with obstacle alpha: {}, alpha_min: {}, alpha_max: {}.".format(alpha, alpha_min, alpha_max))
                else:
                    self.ax = 0
                    self.avoiding = False
        return self.avoiding

    def obstacle_avoidance(self):
        while (self.detect_obstacles() == True):
            self.ax = self.ax + 2
            self.calculate_position()
            self.calculate_velocity()

    def end_direction(self):
        dx = self.end_x - self.x
        dy = self.end_y - self.y
        dz = self.end_z - self.z
        if (dx < 0.1) and (dy <= 0.1) and (dz <= 0.1):
            self.vx = 0
            self.vy = 0
            self.vz = 0
            # print(self.y)
        else:
            Vector_position = np.array([dx, dy, dz])
            Vector_velocity = np.array([self.vx, self.vy, self.vz])
            e = Vector_position / np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            new_V = e * np.linalg.norm(Vector_velocity)
            if self.avoiding == False:
                self.ax = (new_V[0] - self.vx)
                self.ay = (new_V[1] - self.vy)
                self.az = (new_V[2] - self.vz)
            # print(self.ay)
'''

class drone():
    def __init__(self, x, y, z, vx, vy, vz, ax, ay, az, t, obstacle, end_x, end_y, end_z):
        self.position = np.array([x, y, z])
        self.v = np.array([vx, vy, vz])
        self.a = np.array([ax, ay, az])
        self.t = t
        self.dt = 1e-1
        self.obstacle = obstacle
        self.end = np.array([end_x, end_y, end_z])
        self.avoiding = False

    def calculate_velocity(self):
        self.v = self.v + self.a * self.dt

    def calculate_position(self):
        self.position = self.position + self.v * self.dt

    def update_accelerate(self, a):
        self.a = a

    def update_t(self):
        self.t = self.t + self.dt

    def distance(self, ob):
        return np.sqrt((self.position[0] - ob.x) ** 2 + (self.position[1] - ob.y) ** 2)

    def detect_obstacles(self):
        self.avoiding = False
        for ob in self.obstacle:
            drone_pos = self.position[0:2]
            ob1 = np.array([ob.x, ob.y])
            ob2 = np.array([ob.x + ob.length, ob.y])
            ob3 = np.array([ob.x, ob.y + ob.width])
            ob4 = np.array([ob.x + ob.length, ob.y + ob.width])
            vec1 = (ob1 - drone_pos) / np.linalg.norm(ob1 - drone_pos)
            vec2 = (ob2 - drone_pos) / np.linalg.norm(ob2 - drone_pos)
            vec3 = (ob3 - drone_pos) / np.linalg.norm(ob3 - drone_pos)
            vec4 = (ob4 - drone_pos) / np.linalg.norm(ob4 - drone_pos)
            drone_v = self.v[0:2] / np.linalg.norm(self.v[0:2])
            d = np.linalg.norm(ob1 - drone_pos)
            if drone_pos[0] > ob.x and drone_pos[0] < ob.x + ob.length and drone_pos[1] > ob.y and drone_pos[1] < ob.y + ob.width:
                print("obstacle collision at ", ob.x, ob.y)
            if (drone_v[0] > vec1[0] and drone_v[0] > vec2[0] and drone_v[0] > vec3[0] and drone_v[0] > vec4[0]) or \
                    (drone_v[0] < vec1[0] and drone_v[0] < vec2[0] and drone_v[0] < vec3[0] and drone_v[0] < vec4[0]) \
                    or d > 6 or self.position[2] > ob.height:
                self.avoiding = False
            else:
                self.avoiding = True
                print('obstacle detected')
                return ob
        return False

    def obstacle_avoidance(self):
        for i in range(10):
            ob = self.detect_obstacles()
            if ob == False:
                break
            else:
                center = np.array([ob.x + 0.5 * ob.length, ob.y + 0.5 * ob.width])
                radius = 2 * ob.length
                old_v = self.v[0:2]
                d = center - self.position[0:2]
                e = d / np.linalg.norm(d)
                vertical_e = np.array([e[1], -e[0]])
                new_vector = d + vertical_e * radius
                new_vector = new_vector / np.linalg.norm(new_vector)
                new_v = new_vector * np.linalg.norm(old_v)
                # [ax, ay] = (new_v - old_v) / self.dt
                # self.update_accelerate(np.array([ax, ay, 0]))
                # self.calculate_velocity()
                vz = self.v[2]
                self.v = np.array([new_v[0], new_v[1], vz])
                # self.calculate_position()

    def end_direction(self):
        d = self.end - self.position
        if np.linalg.norm(d) < 1e-4:
            self.v = np.array([0, 0, 0])
        else:
            Vector_position = d
            Vector_velocity = self.v
            e = Vector_position / np.linalg.norm(Vector_position)
            new_V = e * np.linalg.norm(Vector_velocity)
            if self.avoiding == False:
                self.a = new_V - self.v
                # print(self.v)

if __name__ == "__main__":
    # define random obstacles
    obs = []
    for i in range(30):
        obs.append(Obstacle(3 + 3 * np.random.randn(), 3 + 3 * np.random.randn(), 0.3, 0.3, 4 * abs(np.random.randn())))

    # define drone cluster
    # drones = [ drone( k * 3 - 4 ,0,0,0,1,0,0,0,0,0, obs, k * 3 - 4 , 10, 1 ) for k in range(5) ]
    q = [[0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [2, 2, 2, 2, 2, 2, 4, 4, 4, 4, 4, 4]]

    x = np.array(2 * q[0], dtype=float)
    y = np.array(q[1] - np.array([5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]), dtype=float)
    z = np.array(q[2], dtype=float)

    drones = [drone(x[k], y[k], z[k], 0, 1, 0, 0, 0, 0, 0, obs, x[k]+ np.random.randn(), y[k] + 10 + np.random.randn(), z[k] + np.random.randn()) for k in
              range(12)]
    px = [[] for k in range(len(drones))]
    py = [[] for k in range(len(drones))]
    pz = [[] for k in range(len(drones))]
    pt = [[] for k in range(len(drones))]

    for j in range(len(drones)):
        print(j)
        drone_cur = drones[j]
        drone_cur.detect_obstacles()
        for i in range(1000):
            # print(i)
            drone_cur.obstacle_avoidance()
            drone_cur.end_direction()
            drone_cur.calculate_velocity()
            drone_cur.calculate_position()
            drone_cur.update_t()
            [x, y, z], [vx, vy, vz], [ax, ay, az], t, [endx, endy, endz]= drone_cur.position, drone_cur.v, drone_cur.a, drone_cur.t, drone_cur.end
            drone_cur = drone(x,y,z,vx,vy,vz,ax,ay,az,t,obs, endx, endy, endz)
            px[j].append(x)
            py[j].append(y)
            pz[j].append(z)
            pt[j].append(t)

    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(projection='3d')
    X = np.array([ob.x for ob in obs])
    Y = np.array([ob.y for ob in obs])
    height = np.array([ob.height for ob in obs])
    bottom = np.zeros_like(X)  # ??????????????????????????????
    height = height
    length = np.array([ob.length for ob in obs])
    width = np.array([ob.width for ob in obs])  # ???????????????????????????
    # ????????????
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