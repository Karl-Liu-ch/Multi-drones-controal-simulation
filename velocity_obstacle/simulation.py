import numpy as np
from multi_robot_plot import plot_robot_and_obstacles
from creat_UAVs import create_obstacles
from UAV import UAV
SIM_TIME = 20.0
NUMBER_OF_TIMESTEPS = int(20.0 / 0.1)
DETECT_NOISE = 0.01
update_frequency = 2

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

    start_3 = np.array([8, 0])
    start_velocity_3 = np.array([0, 0])
    goal_3 = np.array([1, 10])
    robot_3 = UAV(start_3, start_velocity_3, goal_3, 0.5, 2.0)
    robot_state_3 = np.append(start_3, start_velocity_3)
    robot_state_history_3 = np.empty((4, NUMBER_OF_TIMESTEPS))

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

        v_desired = robot_3.compute_desired_velocity()
        if i % update_frequency == 0:
            try:
                input_robot_state1 = np.reshape(robot_state_1, (4, 1))
                input_robot_state1 = DETECT_NOISE * np.random.normal(size=input_robot_state1.shape) + input_robot_state1
                obstate = obstacles[:, i, :] + DETECT_NOISE * np.random.normal(size=obstacles[:, i, :].shape)
                control_vel = robot_3.compute_velocity(
                    np.concatenate((obstate, input_robot_state1), axis=1), v_desired)
            except:
                print("Failed to find a path")
                break
        else:
            control_vel = robot_3.velocity
        robot_state_3 = robot_3.update_state(robot_state_3, control_vel)
        robot_state_history_3[:4, i] = robot_state_3

        #obstacles = np.dstack((obstacles, robot_state_history_2))
        robot_state_history = np.concatenate((robot_state_history_1, robot_state_history_2), axis=1)
    # we can judge the performance of drones by how long they traveled to reach the end position
    print(robot_1.path_length, robot_2.path_length)
    robot_list = [robot_state_history_1, robot_state_history_2, robot_state_history_3]
    plot_robot_and_obstacles(
        robot_list, obstacles, robot_1.robot_radius, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)