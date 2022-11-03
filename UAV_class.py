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
    
    def calculate_velocity(self):
        self.vx = self.vx + self.ax * self.dt
        self.vy = self.vy + self.ay * self.dt
        self.vz = self.vz + self.az * self.dt
    
    def calculate_position(self):
        self.x = self.x + self.vx * self.dt + 0.5 * self.ax * (self.dt ** 2)
        self.y = self.y + self.vy * self.dt + 0.5 * self.ay * (self.dt ** 2)
        self.z = self.z + self.vz * self.dt + 0.5 * self.az * (self.dt ** 2)
    
    def update_t(self):
        self.t = self.t + self.dt
        
drone_1 = drone(0,0,0,0,0,0,5 * np.random.randn(),5 * np.random.randn(),5 * np.random.randn(),0)

px = []
py = []
pz = []
pt = []

for i in range(100):
    drone_1.calculate_velocity()
    drone_1.calculate_position()
    drone_1.update_t()
    x, y, z, vx, vy, vz, t = drone_1.x, drone_1.y, drone_1.z, drone_1.vx, drone_1.vy, drone_1.vz, drone_1.t
    drone_1 = drone(x,y,z,vx,vy,vz,5 * np.random.randn(),5 * np.random.randn(),5 * np.random.randn(),t)
    px.append(x)
    py.append(y)
    pz.append(z)
    pt.append(t)
