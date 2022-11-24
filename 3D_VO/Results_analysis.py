import sys
sys.path.append('../')
from UAV_size_changeable_3d import *
Results = {}
for i in range(4):
    DETECT_NOISE = (i) * 0.02
    for j in range(4):
        update_frequency = j + 1
        for k in range(7):
            Safe_Threshold = 1 + 0.1 * (k + 1)
            PATH = 'UAVs{}_OBS{}_DN{}_UF{}_ST{}/'.format(4, 8, DETECT_NOISE, update_frequency, Safe_Threshold)
            robot_state_history, obs_state_history, robots_radius, obs_radius, obs_heights, results = load_uavs_obs(
                PATH=PATH)
            Results['DN{}_UF{}_ST{}'.format(DETECT_NOISE, update_frequency, Safe_Threshold)] = results
print(Results)