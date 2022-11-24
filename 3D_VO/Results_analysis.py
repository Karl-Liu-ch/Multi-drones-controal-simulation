import sys
sys.path.append('../')
from UAV_size_changeable_3d import *
from matplotlib import pyplot as plt

Results = []
for i in range(10):
    DETECT_NOISE = (i) * 0.02
    for j in range(5):
        update_frequency = j + 1
        for k in range(10):
            Safe_Threshold = 1 + 0.1 * (k + 1)
            PATH = 'UAVs{}_OBS{}_DN{}_UF{}_ST{}/'.format(4, 8, DETECT_NOISE, update_frequency, Safe_Threshold)
            try:
                robot_state_history, obs_state_history, robots_radius, obs_radius, obs_heights, results = load_uavs_obs(
                    PATH=PATH)
                parameter = np.array([DETECT_NOISE, update_frequency, Safe_Threshold])
                Results.append(np.append(results[:], parameter))
            except:
                pass
print(Results)
