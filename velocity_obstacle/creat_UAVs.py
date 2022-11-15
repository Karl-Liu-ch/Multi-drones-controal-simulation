import numpy as np

class Obstacle:
    def __init__(self, position: np.array, velocity: np.array):
        self.position = position
        self.velocity = velocity

def create_obstacles(sim_time, num_timesteps, num_static, num_moving):

    for i in range(num_moving):
        obstacle = Obstacle(np.array([np.random.uniform(0, 10), np.random.uniform(0, 10)]), np.array([np.random.uniform(-1, 1), np.random.uniform(-1, 1)]))
        obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
        try:
            obstacles
        except:
            obstacles = obst
        else:
            obstacles = np.dstack((obstacles, obst))
    

    for i in range(num_static):
        obstacle = Obstacle(np.array([np.random.uniform(0, 10), np.random.uniform(0, 10)]), (0, 0))
        obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
        try:
            obstacles
        except:
            obstacles = obst
        else:
            obstacles = np.dstack((obstacles, obst))

    return obstacles

def create_robot(p0, v, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = np.ones(t.shape[0])
    vx = v[0] * theta
    vy = v[1] * theta
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    return p

