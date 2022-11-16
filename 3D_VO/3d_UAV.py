import numpy as np
from multi_robot_plot import plot_robot_and_obstacles
from creat_UAVs import create_obstacles
import argparse

SIM_TIME = 20.0
NUMBER_OF_TIMESTEPS = int(20.0 / 0.1)
DETECT_NOISE = 0.01
update_frequency = 1
Safe_Threshold = 1.1

class UAV:
    def __init__(self, position: np.array, velocity: np.array, goal: np.array, robot_radius: float, vmax: float):
        self.position = position
        self.velocity = velocity
        self.goal = goal
        self.robot_radius = robot_radius
        self.vmax = vmax
        self.TIMESTEP = 0.1
        self.path_length = 0.0
        self.collide = False
        self.reach_end = False

    def compute_desired_velocity(self):
        disp_vec = (self.goal - self.position)
        norm = np.linalg.norm(disp_vec)
        if norm < self.robot_radius / 5:
            return np.zeros(3)
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
        Dis = []
        angle = []
        v_B = []
        for i in range(number_of_obstacles):
            obstacle = obstacles[:, i]
            pB = obstacle[:3]
            vB = obstacle[3:]
            v_B.append(vB)
            Dis.append(pB - pA)
            absD = np.linalg.norm(pA - pB)
            if Safe_Threshold * 2 * self.robot_radius > absD:
                absD = Safe_Threshold * 2 * self.robot_radius
            angle.append(np.arcsin(Safe_Threshold * 2 * self.robot_radius / absD))

            # VO
        # Create search-space
        th = np.linspace(0, 2 * np.pi, 20)
        th_z = np.linspace(-np.pi, np.pi, 20)
        vel = np.linspace(0, self.vmax, 5)

        vv, thth, thzthz = np.meshgrid(vel, th, th_z)
        # [vv, thth], thzthz = np.meshgrid([vv, thth], th_z)

        vx_sample = (vv * np.cos(thzthz) * np.cos(thth)).flatten()
        vy_sample = (vv * np.cos(thzthz) * np.sin(thth)).flatten()
        vz_sample = (vv * np.sin(thzthz)).flatten()

        v_sample = np.stack((vx_sample, vy_sample, vz_sample))
        v_satisfying_constraints = self.RVO(v_sample, v_B, Dis, angle)

        # Objective function
        size = np.shape(v_satisfying_constraints)[1]
        diffs = v_satisfying_constraints - \
                ((v_desired).reshape(3, 1) @ np.ones(size).reshape(1, size))
        norm = np.linalg.norm(diffs, axis=0)
        min_index = np.where(norm == np.amin(norm))[0][0]
        cmd_vel = (v_satisfying_constraints[:, min_index])
        if self.collide:
            cmd_vel = np.array([0, 0])
            self.velocity = np.array([0, 0])
        return cmd_vel

    def VO(self, v, v_B, Dis, angle):
        v_out = []
        for i in range(np.shape(v)[1]):
            discard = False
            for j in range(len(Dis)):
                vR = v[:, i].T - v_B[j]
                theta = np.arccos(vR.dot(Dis[j]) / (np.linalg.norm(vR) * np.linalg.norm(Dis[j])))
                if theta < angle[j]:
                    discard = True
            if not discard:
                v_out.append(v[:, i])
        return np.array(v_out).T

    def RVO(self, v, v_B, Dis, angle, timestep = 0.1):
        v_out = []
        for i in range(np.shape(v)[1]):
            discard = False
            for j in range(len(Dis)):
                vR = v[:, i].T - v_B[j]
                theta = np.arccos(vR.dot(Dis[j]) / (np.linalg.norm(vR) * np.linalg.norm(Dis[j])))
                if (theta < angle[j]) and ((np.linalg.norm(Dis[j] / timestep - vR)) < Safe_Threshold * 2 * self.robot_radius / timestep):
                    discard = True
            if not discard:
                v_out.append(v[:, i])
        return np.array(v_out).T

    def update_state(self, x, v):
        new_state = np.empty((6))
        new_state[:3] = x[:3] + v * self.TIMESTEP
        new_state[-3:] = v
        old_position = self.position
        self.position = new_state[:3]
        self.path_length += np.linalg.norm(self.position - old_position)
        self.velocity = v
        return new_state

class UAV_cluster():
    def __init__(self, nums_uav):
        start = np.array([0, 0])
        start_velocity = np.array([0, 0])
        goal = np.array([0, 10])
        self.UAVs = [UAV(start, start_velocity, goal, 0.5, 2.0) for i in range(nums_uav)]

    def reset(self, starts, ends):
        assert len(starts) == len(self.UAVs)
        assert len(ends) == len(self.UAVs)
        self.robot_state = []
        self.robot_state_history = []
        for i in range(len(self.UAVs)):
            start = starts[i]
            start_velocity = np.array([0, 0, 0])
            goal = ends[i]
            self.UAVs[i] = UAV(start, start_velocity, goal, 0.5, 2.0)
            robot_state = np.append(start, start_velocity)
            robot_state = robot_state.astype(np.float64)
            robot_state_history = np.empty((6, NUMBER_OF_TIMESTEPS))
            self.robot_state.append(robot_state)
            self.robot_state_history.append(robot_state_history)

    def detect(self, index_of_self):
        step = 0
        robots_states = np.empty((6, 1))
        for i in range(len(self.UAVs)):
            if i == index_of_self:
                pass
            else:
                robot_state = self.robot_state[i]
                robot_state = np.reshape(robot_state, (6, 1))
                robot_state += DETECT_NOISE * np.random.normal(size=robot_state.shape)
                if step == 0:
                    robots_states = robot_state
                else:
                    robots_states = np.concatenate((robots_states, robot_state), axis=1)
                step += 1
        return robots_states

    def update(self, obstacles, time_step):
        for i in range(len(self.UAVs)):
            if time_step % update_frequency == 0:
                # try:
                v_desired = self.UAVs[i].compute_desired_velocity()
                # here I added noise of detection of all obstacles and all other drones
                robots_states = self.detect(i)
                obstate = obstacles[:, time_step, :] + DETECT_NOISE * np.random.normal(
                    size=obstacles[:, time_step, :].shape)
                control_vel = self.UAVs[i].compute_velocity(
                    np.concatenate((obstate, robots_states), axis=1), v_desired)
                if self.UAVs[i].collide == True:
                    control_vel = np.array([0, 0, 0])
                # except:
                #     # if drone cannot find a path, then return task failed.
                #     print("Number {} Failed to find a path".format(i + 1))
                #     break
            else:
                control_vel = self.UAVs[i].velocity
            self.robot_state[i] = self.UAVs[i].update_state(self.robot_state[i], control_vel)
            self.robot_state_history[i][:6, time_step] = self.robot_state[i]

def cal_distance(x, y):
    return np.linalg.norm(x - y)

def Monitor(obstacles, UAVs, timestep):
    for i in range(len(UAVs)-1):
        for j in range(len(UAVs)-i-1):
            d = cal_distance(UAVs[i].position, UAVs[i+j+1].position)
            if d < UAVs[i].robot_radius + UAVs[i+j+1].robot_radius:
                # UAVs[i].collide = True
                # UAVs[j].collide = True
                print("collision of uav {} and uav {}".format(i, i+j+1))
                print(UAVs[i].position)
                print(UAVs[i+j+1].position)
                print(np.linalg.norm(UAVs[i].position-UAVs[i+j+1].position))
        for k in range(obstacles.shape[2]):
            ob = obstacles[:3, timestep, k]
            d = cal_distance(UAVs[i].position, ob)
            if d < UAVs[i].robot_radius + 0.5:
                # UAVs[i].collide = True
                print("collision of uav {}".format(i))

def simulate(filename):
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS, 3, 3)
    starts = [np.array([6, 0, 0])]
    starts.append(np.array([3, 0, 0]))
    starts.append(np.array([8, 0, 0]))
    goals = [np.array([3, 10, 10])]
    goals.append(np.array([6, 10, 10]))
    goals.append(np.array([1, 10, 10]))
    UAVs = UAV_cluster(3)
    UAVs.reset(starts, goals)

    for i in range(NUMBER_OF_TIMESTEPS):
        # Update frequency parameter
        UAVs.update(obstacles, i)
        Monitor(obstacles, UAVs.UAVs, i)
        # print(UAVs.robot_state_history[0][:6,i])
    plot_robot_and_obstacles(
        UAVs.robot_state_history, obstacles, UAVs.UAVs[0].robot_radius, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filename", help="filename, in case you want to save the animation")
    args = parser.parse_args()
    simulate(args.filename)

