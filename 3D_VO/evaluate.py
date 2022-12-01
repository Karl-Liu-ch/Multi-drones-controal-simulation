import sys
sys.path.append('/')
import numpy as np
from multi_robot_plot import plot_robot_and_obstacles
from creat_UAVs_1 import create_obstacles,create_fixed_obstacles_scenario, Obstacles
import argparse
from visualization import visualization
import os
from UAV_size_changeable_3d import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filename", help="filename, in case you want to save the animation")
    args = parser.parse_args()
    ROOT = 'simulation_results/'
    PATH = 'UAVs{}_OBS{}_DN{}_UF{}_ST{}/'.format(4, 8, 0.06, 2, 1.4)
    for i in range(10):
        DETECT_NOISE = (i) * 0.02
        for j in range(5):
            update_frequency = j + 1
            print("noise: {}, update frequency: {}".format(DETECT_NOISE, update_frequency))
            for k in range(10):
                Safe_Threshold = 1 + 0.1 * (k + 1)
                PATH = 'UAVs{}_OBS{}_DN{}_UF{}_ST{}/'.format(4, 8, DETECT_NOISE, update_frequency, Safe_Threshold)
                # try:
                #     os.mkdir('simulation_results/' + PATH)
                # except:
                #     pass
                # try:
                #     robot_state_history, obs_state_history, robots_radius, obs_radius, obs_heights, results = load_uavs_obs(
                #         PATH=PATH)
                #     print("simulation found")
                # except:
                #     UAVs, OBS = simulate(DETECT_NOISE, update_frequency, Safe_Threshold)
                #     print("simulating")
                # print("saved noise {} update frequency {} safe threshold {}".format(DETECT_NOISE,
                #                                                                     update_frequency, Safe_Threshold))
                robot_state_history, obs_state_history, robots_radius, obs_radius, obs_heights, results = load_uavs_obs(
                    PATH=PATH)
                # plot_robot_and_obstacles(
                #     robot_state_history, obs_state_history, robots_radius, obs_radius, NUMBER_OF_TIMESTEPS, SIM_TIME,
                #     args.filename)
                # visualization(robot_state_history, robots_radius, obs_state_history, obs_radius, obs_heights,
                #               NUMBER_OF_TIMESTEPS)
                # print(results)
    # robot_state_history, obs_state_history, robots_radius, obs_radius, obs_heights, results = load_uavs_obs(PATH=PATH)
    # plot_robot_and_obstacles(
    #     robot_state_history, obs_state_history, robots_radius, obs_radius, NUMBER_OF_TIMESTEPS, SIM_TIME, args.filename)
    # visualization(robot_state_history, robots_radius, obs_state_history, obs_radius, obs_heights, NUMBER_OF_TIMESTEPS)
    # print(results)