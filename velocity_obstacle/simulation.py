import numpy as np
from multi_robot_plot import plot_robot_and_obstacles
from creat_UAVs import create_obstacles
from UAV import UAV_cluster, Monitor
import argparse
SIM_TIME = 20.0
NUMBER_OF_TIMESTEPS = int(20.0 / 0.1)
DETECT_NOISE = 0.01
update_frequency = 2

def simulate(filename):
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS, 3, 3)
    starts = [np.array([6, 0])]
    starts.append(np.array([3, 0]))
    starts.append(np.array([8, 0]))
    goals = [np.array([3, 10])]
    goals.append(np.array([6, 10]))
    goals.append(np.array([1, 10]))
    UAVs = UAV_cluster(3)
    UAVs.reset(starts, goals)

    for i in range(NUMBER_OF_TIMESTEPS):
        # Update frequency parameter
        UAVs.update(obstacles, i)
        Monitor(obstacles, UAVs.UAVs, i)
    plot_robot_and_obstacles(
        UAVs.robot_state_history, obstacles, UAVs.UAVs[0].robot_radius, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filename", help="filename, in case you want to save the animation")
    args = parser.parse_args()
    simulate(args.filename)