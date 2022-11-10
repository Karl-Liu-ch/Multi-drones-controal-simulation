import numpy as np

class drone():
    def __init__(self):
        self.position = np.array([0, 0, 0])
        self.start = np.array([0, 0, 0])
        self.t = 0
        self.dt = 1
        self.end = np.array([10, 10, 10])
        self.avoiding = False
        
    def decode_action(self, action):
        if action == 0:
            return np.array([0, 0, 0])
        elif action == 1:
            return np.array([1, 0, 0])
        elif action == 2:
            return np.array([0, 1, 0])
        elif action == 3:
            return np.array([0, 0, 1])
        elif action == 4:
            return np.array([-1, 0, 0])
        elif action == 5:
            return np.array([0, -1, 0])
        elif action == 6:
            return np.array([0, 0, -1])
        
    def next_position(self, action):
        self.position = self.position + self.decode_action(action) * self.dt
        
    def update_t(self):
        self.t = self.t + self.dt
    
    def cal_distance(self, a, b):
        return np.linalg.norm(a - b)
    
    def set_position(self, position):
        self.position = position
    
    def set_end(self, end):
        self.end = end
    
    def reset(self):
        self.t = 0
        self.set_position(np.array([0, 0, 0]))
        self.set_end(np.array([10, 10, 10]))
        return np.append(self.position, np.append(self.end, self.t))
    
    def out_map(self):
        if self.position[0] > 15 or self.position[1] > 15 or self.position[2] > 15 or self.position[0] < -15 or self.position[1] < -15 or self.position[2] < 0:
            return True
        else:
            return False
    
    def step(self, action):
        done = False
        self.next_position(action)
        self.t = self.t + self.dt
        state = np.append(self.position, np.append(self.end, self.t))
        reward = - (self.cal_distance(self.position, self.end)) / self.cal_distance(self.end, self.start) - self.t / 50.0
        # print(self.cal_distance(self.position, self.end))
        if self.cal_distance(self.position, self.end) == 0:
            done = True
            reward = reward + 300
            print("success")
        # elif self.out_map():
        #     done = True
        #     reward = reward - 3
            # print(self.cal_distance(self.position, self.end))
        return state, reward, done, True
        
