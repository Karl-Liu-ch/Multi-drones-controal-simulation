import numpy as np

class drone():
    def __init__(self, x, y, z, vx, vy, vz, ax, ay, az, t):
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
        self.dt = 1e-2
    
    def calculate_velocity(self, ax, ay, az):
        self.ax, self.ay, self.az = ax, ay, az
        self.vx = self.vx + self.ax * self.dt
        self.vy = self.vy + self.ay * self.dt
        self.vz = self.vz + self.az * self.dt
        print(self.ay)
        print(self.vy)
    def calculate_position(self):
        self.x = self.x + self.vx * self.dt + 0.5 * self.ax * (self.dt ** 2)
        self.y = self.y + self.vy * self.dt + 0.5 * self.ay * (self.dt ** 2)
        self.z = self.z + self.vz * self.dt + 0.5 * self.az * (self.dt ** 2)

    def update_t(self):
        self.t = self.t + self.dt

# drone_1 = drone(0,0,0,0,0,0,5 * np.random.randn(),5 * np.random.randn(),5 * np.random.randn(),0)
#
# drones = [drone(0,0,0,0,0,0,5 * np.random.randn(),5 * np.random.randn(),5 * np.random.randn(),0) for k in range(5)]

# px = [[] for k in range(len(drones))]
# py = [[] for k in range(len(drones))]
# pz = [[] for k in range(len(drones))]
# pt = [[] for k in range(len(drones))]

# print(px)
# for j in range(len(drones)):
#     drone_cur = drones[j]
#     for i in range(100):
#         drone_cur.calculate_velocity()
#         drone_cur.calculate_position()
#         drone_cur.update_t()
#         x, y, z, vx, vy, vz, t = drone_cur.x, drone_cur.y, drone_cur.z, drone_cur.vx, drone_cur.vy, drone_cur.vz, drone_cur.t
#         drone_cur = drone(x,y,z,vx,vy,vz,5 * np.random.randn(),5 * np.random.randn(),5 * np.random.randn(),t)
#         px[j].append(x)
#         py[j].append(y)
#         pz[j].append(z)
#         pt[j].append(t)
