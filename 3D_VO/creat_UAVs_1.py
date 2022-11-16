import numpy as np

class Obstacle:
    def __init__(self, position: np.array, velocity: np.array):
        self.position = position
        self.velocity = velocity

def create_obstacles(sim_time, num_timesteps, num_static, num_moving):

    for i in range(num_moving):
        obstacle = Obstacle(np.array([np.random.uniform(0, 10), np.random.uniform(0, 10)+1, 0]),
                            np.array([np.random.uniform(-1, 1), np.random.uniform(-1, 1), 0]))
        obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps, 1)
        try:
            obstacles
        except:
            obstacles = obst
        else:
            obstacles = np.dstack((obstacles, obst))
    

    for i in range(num_static):
        obstacle = Obstacle(np.array([np.random.uniform(0, 10), np.random.uniform(0, 10)+1, 0]), (0, 0, 0))
        obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps, 1)
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
    vz = v[2] * theta
    v = np.stack([vx, vy, vz])
    p0 = p0.reshape((3, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    return p

def create_fixed_obstacles_scenario(sim_time, num_timesteps,scenario):
    Obs = []
    if scenario == 1: #oneway
        y_o = 5
        num_o = 12
        for i in range(num_o): 
            # if i != num_o/2 and i!=num_o/2+1:
            obstacle = Obstacle(np.array([i-0.5, y_o, 2]), (0, 0, 0))
            Obs.append(obstacle)
            obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps, 1)
            try:
                obstacles
            except:
                obstacles = obst
            else:
                obstacles = np.dstack((obstacles, obst))
            obstacle = Obstacle(np.array([i - 0.5, y_o, 1]), (0, 0, 0))
            obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps,
                                                                                                       1)
            obstacles = np.dstack((obstacles, obst))
            obstacle = Obstacle(np.array([i - 0.5, y_o, 0]), (0, 0, 0))
            obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps,
                                                                                                       1)
            obstacles = np.dstack((obstacles, obst))
    elif scenario == 2: #obs split from middle 
        x_0 = 10
        for i in range(9):
            for k in range (2):
                obstacle = Obstacle(np.array([x_0, i-0.5, 0]), np.array([np.power(-1,k)*i*0.2, 0, 0]))
                obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps, 1)
                try:
                    obstacles
                except:
                    obstacles = obst
                else:
                    obstacles = np.dstack((obstacles, obst))
    elif scenario == 3: # obs narrows the path
        v_0 = 0.6
        num_o = 18
        x_left = -2
        x_right = 22
        for i in range(num_o):
            obstacle = Obstacle(np.array([x_right, i-9.5,0]), np.array([-v_0, 0, 0]))
            obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps, 1)
            try:
                obstacles
            except:
                obstacles = obst
            else:
                obstacles = np.dstack((obstacles, obst))
            obstacle = Obstacle(np.array([x_left, i-9.5,0]), np.array([v_0, 0,0]))
            obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps, 1)
            try:
                obstacles
            except:
                obstacles = obst
            else:
                obstacles = np.dstack((obstacles, obst))
    elif scenario == 4: # sth like a forest
        x_0 = 0.5
        x_1 = 1
        num_h = 18
        num_v = 5
        k = 1 
        for i in range(num_h):
            for l in range(num_v):
                k = l+1
                obstacle = Obstacle(np.array([x_0+(i-1)*4+k, k*20/(num_v+1),0]), np.array([0, 0, 0]))
                obst = create_robot(obstacle.position, obstacle.velocity, sim_time, num_timesteps).reshape(6, num_timesteps, 1)
                try:
                    obstacles
                except:
                    obstacles = obst
                else:
                    obstacles = np.dstack((obstacles, obst))
    return obstacles, Obs