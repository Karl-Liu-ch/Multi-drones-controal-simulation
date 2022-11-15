import numpy as np
from multi_robot_plot import plot_robot_and_obstacles
from creat_UAVs import create_obstacles
import argparse
SIM_TIME = 20.0
NUMBER_OF_TIMESTEPS = int(20.0 / 0.1)
DETECT_NOISE = 0.01
update_frequency = 2

class UAV:
    def __init__(self, position: np.array, velocity: np.array, goal: np.array, robot_radius: float, vmax: float):
        self.position = position
        self.velocity = velocity
        self.goal = goal
        self.robot_radius = robot_radius
        self.vmax = vmax
        self.TIMESTEP = 0.1
        self.path_length = 0.0
    
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
        old_position = self.position
        self.position = new_state[:2]
        self.path_length += np.linalg.norm(self.position - old_position)
        self.velocity = v
        return new_state

def simulate(filename):
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS, 3, 3)

    start_1 = np.array([6, 0])
    start_velocity_1 = np.array([0,0])
    goal_1 = np.array([3, 10])
    robot_1 = UAV(start_1, start_velocity_1, goal_1, 0.5, 2.0)
    robot_state_1 = np.append(start_1, start_velocity_1)
    robot_state_history_1 = np.empty((4, NUMBER_OF_TIMESTEPS))

    start_2 = np.array([3, 0])
    start_velocity_2 = np.array([0, 0])
    goal_2 = np.array([6, 10])
    robot_2 = UAV(start_2, start_velocity_2, goal_2, 0.5, 2.0)
    robot_state_2 = np.append(start_2, start_velocity_2)
    robot_state_history_2 = np.empty((4, NUMBER_OF_TIMESTEPS))

    for i in range(NUMBER_OF_TIMESTEPS):
        # Update frequency parameter
        if i % update_frequency == 0:
            v_desired = robot_1.compute_desired_velocity()
            try:
                # here I added noise of detection of all obstacles and all other drones
                input_robot_state2 = np.reshape(robot_state_2, (4, 1))
                input_robot_state2 = DETECT_NOISE * np.random.normal(size=input_robot_state2.shape) + input_robot_state2
                obstate = obstacles[:, i, :] + DETECT_NOISE * np.random.normal(size=obstacles[:, i, :].shape)
                control_vel = robot_1.compute_velocity(
                    np.concatenate((obstate,input_robot_state2), axis=1),v_desired)
            except:
                # if drone cannot find a path, then return task failed.
                print("Failed to find a path")
                break
        else:
            control_vel = robot_1.velocity
        robot_state_1 = robot_1.update_state(robot_state_1, control_vel)
        robot_state_history_1[:4, i] = robot_state_1

        v_desired = robot_2.compute_desired_velocity()
        if i % update_frequency == 0:
            try:
                input_robot_state1 = np.reshape(robot_state_1, (4, 1))
                input_robot_state1 = DETECT_NOISE * np.random.normal(size=input_robot_state1.shape) + input_robot_state1
                obstate = obstacles[:, i, :] + DETECT_NOISE * np.random.normal(size=obstacles[:, i, :].shape)
                control_vel = robot_2.compute_velocity(
                    np.concatenate((obstate, input_robot_state1), axis=1), v_desired)
            except:
                print("Failed to find a path")
                break
        else:
            control_vel = robot_2.velocity
        robot_state_2 = robot_2.update_state(robot_state_2, control_vel)
        robot_state_history_2[:4, i] = robot_state_2

        #obstacles = np.dstack((obstacles, robot_state_history_2))
        robot_state_history = np.concatenate((robot_state_history_1, robot_state_history_2), axis=1)
    # we can judge the performance of drones by how long they traveled to reach the end position
    print(robot_1.path_length, robot_2.path_length)
    robot_list = [robot_state_history_1, robot_state_history_2]
    plot_robot_and_obstacles(
        robot_list, obstacles, robot_1.robot_radius, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filename", help="filename, in case you want to save the animation")
    args = parser.parse_args()
    simulate(args.filename)