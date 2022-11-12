import numpy as np

class Obstacle:
    def __init__(self, position: np.array, velocity: np.array):
        self.position = position
        self.velocity = velocity

def create_obstacles(sim_time, num_timesteps):
    obstacle = Obstacle(np.array([0,0]), np.array([1,1]))
    obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = obst

    obstacle = Obstacle(np.array([5, 5]), np.array([0.2, 0.6]))
    obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
    obstacles = np.dstack((obstacles, obst))

    obstacle = Obstacle(np.array([6, 6]), np.array([0, 0]))
    obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(4, num_timesteps, 1)
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
# obstacle = Obstacle(np.array([0,0]), np.array([1,1]))
# p = create_robot(obstacle.position, obstacle.velocity, 5.0, 50)
