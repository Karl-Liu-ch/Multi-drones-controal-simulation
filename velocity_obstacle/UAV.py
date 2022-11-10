import numpy as np
from multi_robot_plot import plot_robot_and_obstacles
from creat_obstacles import create_obstacles
import argparse
SIM_TIME = 5.0
NUMBER_OF_TIMESTEPS = int(5.0 / 0.1)

class UAV:
    def __init__(self, position: np.array, velocity: np.array, goal: np.array, robot_radius: float, vmax: float):
        self.position = position
        self.velocity = velocity
        self.goal = goal
        self.robot_radius = robot_radius
        self.vmax = vmax
        self.TIMESTEP = 0.1
    
    def compute_desired_velocity(self):
        disp_vec = (self.goal - self.position)
        norm = np.linalg.norm(disp_vec)
        if norm < self.robot_radius / 5:
            return np.zeros(2)
        disp_vec = disp_vec / norm
        np.shape(disp_vec)
        desired_vel = self.vmax * disp_vec
        return desired_vel
    
    def compute_velocity(self, obstacles, v_desired):
        pA = self.position
        vA = self.velocity
        # Compute the constraints
        # for each velocity obstacles
        number_of_obstacles = np.shape(obstacles)[1]
        Amat = np.empty((number_of_obstacles * 2, 2))
        bvec = np.empty((number_of_obstacles * 2))
        for i in range(number_of_obstacles):
            obstacle = obstacles[:, i]
            pB = obstacle[:2]
            vB = obstacle[2:]
            dispBA = pA - pB
            distBA = np.linalg.norm(dispBA)
            thetaBA = np.arctan2(dispBA[1], dispBA[0])
            if 2.2 * self.robot_radius > distBA:
                distBA = 2.2*self.robot_radius
            phi_obst = np.arcsin(2.2*self.robot_radius/distBA)
            phi_left = thetaBA + phi_obst
            phi_right = thetaBA - phi_obst

            # VO
            translation = vB
            Atemp, btemp = self.create_constraints(translation, phi_left, "left")
            Amat[i*2, :] = Atemp
            bvec[i*2] = btemp
            Atemp, btemp = self.create_constraints(translation, phi_right, "right")
            Amat[i*2 + 1, :] = Atemp
            bvec[i*2 + 1] = btemp

        # Create search-space
        th = np.linspace(0, 2*np.pi, 20)
        vel = np.linspace(0, self.vmax, 5)

        vv, thth = np.meshgrid(vel, th)

        vx_sample = (vv * np.cos(thth)).flatten()
        vy_sample = (vv * np.sin(thth)).flatten()

        v_sample = np.stack((vx_sample, vy_sample))

        v_satisfying_constraints = self.check_constraints(v_sample, Amat, bvec)

        # Objective function
        size = np.shape(v_satisfying_constraints)[1]
        diffs = v_satisfying_constraints - \
            ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
        norm = np.linalg.norm(diffs, axis=0)
        min_index = np.where(norm == np.amin(norm))[0][0]
        cmd_vel = (v_satisfying_constraints[:, min_index])

        return cmd_vel

    def check_constraints(self, v_sample, Amat, bvec):
        length = np.shape(bvec)[0]

        for i in range(int(length/2)):
            v_sample = self.check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])

        return v_sample
    
    def check_inside(self, v, Amat, bvec):
        v_out = []
        for i in range(np.shape(v)[1]):
            if not ((Amat @ v[:, i] < bvec).all()):
                v_out.append(v[:, i])
        return np.array(v_out).T


    def create_constraints(self, translation, angle, side):
        # create line
        origin = np.array([0, 0, 1])
        point = np.array([np.cos(angle), np.sin(angle)])
        line = np.cross(origin, point)
        line = self.translate_line(line, translation)

        if side == "left":
            line *= -1

        A = line[:2]
        b = -line[2]

        return A, b

    def translate_line(self, line, translation):
        matrix = np.eye(3)
        matrix[2, :2] = -translation[:2]
        return matrix @ line

    def update_state(self, x, v):
        new_state = np.empty((4))
        new_state[:2] = x[:2] + v * self.TIMESTEP
        new_state[-2:] = v
        return new_state

class Obstacle:
    def __init__(self, position: np.array, velocity: np.array):
        self.position = position
        self.velocity = velocity

def simulate(filename):
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)

    start = np.array([5, 0])
    start_velocity = np.array([0,0])
    goal = np.array([5, 10])
    robot = UAV(start, start_velocity, goal, 0.5, 2.0)

    robot_state = np.append(start, start_velocity)
    robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))
    for i in range(NUMBER_OF_TIMESTEPS):
        v_desired = robot.compute_desired_velocity()
        control_vel = robot.compute_velocity(obstacles[:, i, :], v_desired)
        robot_state = robot.update_state(robot_state, control_vel)
        robot_state_history[:4, i] = robot_state

    plot_robot_and_obstacles(
        robot_state_history, obstacles, robot.robot_radius, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filename", help="filename, in case you want to save the animation")
    args = parser.parse_args()
    simulate(args.filename)