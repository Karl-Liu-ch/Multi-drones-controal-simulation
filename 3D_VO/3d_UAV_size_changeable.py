import numpy as np
from multi_robot_plot import plot_robot_and_obstacles
from creat_UAVs_1 import create_obstacles,create_fixed_obstacles_scenario
import argparse
from visualization import visualization

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

    def compute_velocity(self, obstacles, robots, v_desired):
        pA = self.position
        vA = self.velocity
        # Compute the constraints
        # for each velocity obstacles
        number_of_obstacles = np.shape(obstacles)[1]
        Dis = []
        angle = []
        v_B = []
        Dis_xy_ob = []
        Dis_yz_ob = []
        angle_xy_ob = []
        angle_yz_ob = []
        v_B_xy = []
        v_B_yz = []
        # Create search-space
        th = np.linspace(0, 2 * np.pi, 20)
        th_z = np.linspace(-np.pi, np.pi, 20)
        vel = np.linspace(0, self.vmax, 5)
        vv, thth, thzthz = np.meshgrid(vel, th, th_z)
        vx_sample = (vv * np.cos(thzthz) * np.cos(thth)).flatten()
        vy_sample = (vv * np.cos(thzthz) * np.sin(thth)).flatten()
        vz_sample = (vv * np.sin(thzthz)).flatten()
        v_sample = np.stack((vx_sample, vy_sample, vz_sample))
        for i in range(number_of_obstacles):
            obstacle = obstacles[:, i]
            pB = obstacle[:3]
            vB = obstacle[3:]
            pB_xy = obstacle[:2]
            pB_yz = obstacle[1:3]
            vB_xy = obstacle[3:5]
            vB_yz = obstacle[4:6]
            v_B_xy.append(vB_xy), v_B_yz.append(vB_yz)
            v_B.append(vB)
            Dis_xy = pB_xy - pA[:2]
            Dis_yz = pB_yz - pA[1:3]
            Dis_xy_ob.append(Dis_xy)
            Dis_yz_ob.append(Dis_yz)
            abdDis_xy = np.linalg.norm(Dis_xy)
            abdDis_yz = np.linalg.norm(Dis_yz)
            if Safe_Threshold * 2 * self.robot_radius > abdDis_xy:
                abdDis_xy = Safe_Threshold * 2 * self.robot_radius
            angle_xy_ob.append(np.arcsin(Safe_Threshold * 2 * self.robot_radius / abdDis_xy))
            if Safe_Threshold * 2 * self.robot_radius > abdDis_yz:
                abdDis_yz = Safe_Threshold * 2 * self.robot_radius
            angle_yz_ob.append(np.arcsin(Safe_Threshold * 2 * self.robot_radius / abdDis_yz))

        for robot in robots:
            pB = robot.position.astype(np.float64)
            vB = robot.velocity.astype(np.float64)
            v_B.append(vB)
            Dis.append(pB - pA)
            absD = np.linalg.norm(pA - pB)
            if Safe_Threshold * (self.robot_radius + robot.robot_radius) > absD:
                absD = Safe_Threshold * (self.robot_radius + robot.robot_radius)
            angle.append(np.arcsin(Safe_Threshold * (self.robot_radius + robot.robot_radius) / absD))
            # VO
        v_satisfying_constraints = self.VO(v_sample, v_B, Dis, angle)
        v_satisfying_constraints = self.VO_ob(v_satisfying_constraints, v_B_xy, v_B_yz,
                                              Dis_xy_ob, Dis_yz_ob, angle_xy_ob, angle_yz_ob)

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

    def VO_ob(self, v, v_B_xy, v_B_yz, Dis_xy, Dis_yz, angle_xy, angle_yz):
        v_out = []
        for i in range(np.shape(v)[1]):
            discard = False
            for j in range(len(Dis_xy)):
                vR_xy = v[:2, i].T - v_B_xy[j]
                vR_yz = v[-2:, i].T - v_B_yz[j]
                theta_xy = np.arccos(vR_xy.dot(Dis_xy[j]) / (np.linalg.norm(vR_xy) * np.linalg.norm(Dis_xy[j])))
                theta_yz = np.arccos(vR_yz.dot(Dis_yz[j]) / (np.linalg.norm(vR_yz) * np.linalg.norm(Dis_yz[j])))
                alpha = np.arctan2(Dis_yz[j][1], Dis_yz[j][0])
                alpha_v = np.arctan2(vR_yz[1], vR_yz[0])
                if (theta_xy < angle_xy[j]) and ((theta_yz < angle_yz[j]) or (alpha_v < alpha)):
                    discard = True
                    break
            if not discard:
                v_out.append(v[:, i])
        return np.array(v_out).T

    def VO(self, v, v_B, Dis, angle):
        v_out = []
        for i in range(np.shape(v)[1]):
            discard = False
            for j in range(len(Dis)):
                vR = v[:, i].T - v_B[j]
                theta = np.arccos(vR.dot(Dis[j]) / (np.linalg.norm(vR) * np.linalg.norm(Dis[j])))
                if theta < angle[j]:
                    discard = True
                    break
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
        robots = []
        for i in range(len(self.UAVs)):
            if i == index_of_self:
                pass
            else:
                robots.append(self.UAVs[i])
        return robots

    def update(self, obstacles, time_step):
        for i in range(len(self.UAVs)):
            if time_step % update_frequency == 0:
                # try:
                v_desired = self.UAVs[i].compute_desired_velocity()
                # here I added noise of detection of all obstacles and all other drones
                robots = self.detect(i)
                obstate = obstacles[:, time_step, :] + DETECT_NOISE * np.random.normal(
                    size=obstacles[:, time_step, :].shape)
                control_vel = self.UAVs[i].compute_velocity(obstate, robots, v_desired)
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
    obstacles, OBS = create_fixed_obstacles_scenario(SIM_TIME, NUMBER_OF_TIMESTEPS, 1)
    starts = [np.array([1, 10, 0])]
    starts.append(np.array([5, 0, 0]))
    starts.append(np.array([9, 0, 0]))
    starts.append(np.array([1, 0, 0]))
    goals = [np.array([9, 0, 0])]
    goals.append(np.array([5, 10, 0]))
    goals.append(np.array([1, 10, 0]))
    goals.append(np.array([9, 10, 0]))
    UAVs = UAV_cluster(len(goals))
    UAVs.reset(starts, goals)

    for i in range(NUMBER_OF_TIMESTEPS):
        # Update frequency parameter
        UAVs.update(obstacles, i)
        Monitor(obstacles, UAVs.UAVs, i)
        print(UAVs.robot_state[0])
    plot_robot_and_obstacles(
        UAVs.robot_state_history, obstacles, UAVs.UAVs[0].robot_radius, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)
    visualization(UAVs.robot_state_history, OBS)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filename", help="filename, in case you want to save the animation")
    args = parser.parse_args()
    simulate(args.filename)

