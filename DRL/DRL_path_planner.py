import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from Obstacle_Class import Obstacle

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

    def reset_t(self):
        for drone in self.drones:
            drone.t = 0.0

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
        self.set_va()
        self.set_start()
        self.set_end()
        self.reset_t()
        state = self.get_state()
        state = np.append(state, self.obstate)
        return state
    
    def step(self, action):
        state = []
        dones = []
        reward = 0
        for i in range(len(self.drones)):
            a = action[i]
            done, d = self.drones[i].end_direction()
            d_start = self.calDistance(d, self.start_point[i])
            if done:
                self.drones[i].set_va()
                reward = reward - d / (d_start + 1.0) - self.drones[i].t / 10.0 + self.collsion() * 1e2
                # reward = reward - d + 10.0 / (d + 1.0) - self.drones[i].t + 10.0 / (self.drones[i].t + 1.0)
                state.append(self.drones[i].get_state())
            else:
                self.drones[i].update_accelerate(a)
                self.drones[i].calculate_velocity()
                self.drones[i].calculate_position()
                self.drones[i].update_t()
                reward = reward - d / (d_start + 1.0) - self.drones[i].t / 10.0 + self.collsion() * 1e2
                # reward = reward - d + 10.0 / (d + 1.0) - self.drones[i].t + 10.0 / (self.drones[i].t + 1.0)
                state.append(self.drones[i].get_state())
            dones.append(done)
        state = np.array(state).flatten()
        state = np.append(state, self.obstate)
        if all(dones) == True:
            Done = True
        else:
            Done = False
        return state, reward, Done
    
class drone():
    def __init__(self):
        self.position = np.array([0,0,0])
        self.v = np.array([0,0,0])
        self.a = np.array([0,0,0])
        self.t = 0
        self.dt = 1e-1
        # self.obstacle = obstacle
        self.end = np.array([10 * np.random.randn(),10 * np.random.randn(),abs(10 * np.random.randn())])
        self.avoiding = False
    
    def set_end(self, end):
        self.end = end
        
    def set_start(self, start):
        self.position = start
    
    def set_va(self):
        self.v = np.array([0,0,0])
        self.a = np.array([0,0,0])

    def get_state(self):
        state = np.append(self.position, np.append(self.v, np.append(self.a, self.end)))
        state = np.insert(state, 12, self.t)
        return state

    def calculate_velocity(self):
        V = self.v + self.a * self.dt
        if np.linalg.norm(V) > 20:
            self.v = self.v
        else:
            self.v = V

    def calculate_position(self):
        self.position = self.position + self.v * self.dt

    def update_accelerate(self, a):
        self.a = a

    def update_t(self):
        self.t = self.t + self.dt

    def distance(self, ob):
        return np.sqrt((self.position[0] - ob.x) ** 2 + (self.position[1] - ob.y) ** 2)
    
    def step(self, a):
        done, d = self.end_direction()
        if done:
            self.v = np.array([0,0,0])
            self.a = np.array([0,0,0])
            reward = - d + 10.0 / (d + 1.0) - self.t + 10.0 / (self.t + 1.0)
            state = np.append(self.position, np.append(self.v, np.append(self.a, self.end)))
            state = np.insert(state, 12, self.t)
            return state, reward, done
        else:
            self.update_accelerate(a)
            self.calculate_velocity()
            self.calculate_position()
            self.update_t()
            reward = - d + 10.0 / (d + 1.0) - self.t + 10.0 / (self.t + 1.0)
            state = np.append(self.position, np.append(self.v, np.append(self.a, self.end)))
            state = np.insert(state, 12, self.t)
        return state, reward, done
        
    def end_direction(self):
        d = self.end - self.position
        # if np.linalg.norm(d) < 1e-2:
        if np.linalg.norm(d) < 1e-2 or self.t > 10:
            return True, np.linalg.norm(d)
        else:
            return False, np.linalg.norm(d)
    
    def distance(self):
        return np.linalg.norm(self.position - self.end)
    
    def reset(self):
        self.t = 0
        self.position = np.array([0,0,0])
        self.v = np.array([0,0,0])
        self.a = np.array([0,0,0])
        state = np.append(self.position, np.append(self.v, np.append(self.a, self.end)))
        state = np.insert(state, 9, self.t)
        return state

if __name__ == "__main__":
    pass