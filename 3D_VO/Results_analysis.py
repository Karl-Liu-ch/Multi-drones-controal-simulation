import os
import sys
sys.path.append('../')
from UAV_size_changeable_3d import *
import pandas as pd
import numpy as np
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
                robot_state_history, obs_state_history, robots_radius, obs_radius, obs_heights, results, reachend = load_uavs_obs(
                    PATH=PATH)
                parameter = np.array([DETECT_NOISE, update_frequency, Safe_Threshold])
                result = np.append(results[:], parameter)
                Results.append(np.append(result, reachend))
            except:
                pass
Results = np.array(Results)
d = {}
d['path_length'] = Results[:,0]
d['self_collision'] = Results[:,1]
d['obstacles_collision'] = Results[:,2]
d['failed'] = Results[:,3]
d['DETECT_NOISE'] = Results[:,4]
d['update_frequency'] = Results[:,5]
d['Safe_Threshold'] = Results[:,6]
d['reachend']  = np.array([Results[i, 7:].shape[0] - Results[i, 7:].sum() for i in range(Results.shape[0])])
DF = pd.DataFrame(data=d)

def plot_noise_results(update_frequency = 1.0, Safe_Threshold = 1.1):
    noise = []
    path_length = []
    self_collision = []
    obs_collision = []
    faileds = []
    reachends = []
    for i in DF.index:
        # print(DF.loc[i, 'update_frequency'])
        if DF.loc[i, 'update_frequency'] == update_frequency:
            # print(DF.loc[i, 'Safe_Threshold'])
            if DF.loc[i, 'Safe_Threshold'] == Safe_Threshold:
                x = DF.loc[i, 'DETECT_NOISE']
                y1 = DF.loc[i, 'path_length']
                y2 = DF.loc[i, 'self_collision']
                y3 = DF.loc[i, 'obstacles_collision']
                y4 = DF.loc[i, 'failed']
                y5 = DF.loc[i, 'reachend']
                reachends.append(y5)
                faileds.append(y4)
                noise.append(x)
                path_length.append(y1)
                self_collision.append(y2)
                obs_collision.append(y3)
    noise = np.array(noise)
    path_length = np.array(path_length)
    self_collision = np.array(self_collision)
    obs_collision = np.array(obs_collision)
    plt.plot(noise, path_length, 'ro-', label='Distance')
    plt.plot(noise, self_collision, 'b*-', label='self collision')
    plt.plot(noise, obs_collision, 'b^-', label = 'osbtacle collision')
    # plt.plot(noise, faileds, 'g^-', label='failed')
    plt.plot(noise, reachends, 'g^-', label='failed to reachend')
    plt.title('update_frequency={}_Safe_Threshold={}'.format(update_frequency, Safe_Threshold))
    plt.xlabel('DETECT_NOISE')
    plt.ylabel('Distance')
    plt.legend()
    plt.savefig('Figures/DETECT_NOISE/'
                'DETECT_NOISE_results_update_frequency={}_Safe_Threshold={}.png'.format(update_frequency, Safe_Threshold))
    plt.close()
    plt.plot(noise, path_length, 'ro-', label='Distance')
    plt.title('update_frequency={}_Safe_Threshold={}'.format(update_frequency, Safe_Threshold))
    plt.xlabel('DETECT_NOISE')
    plt.ylabel('Distance')
    plt.savefig('Figures/DETECT_NOISE/Distance/'
                'DETECT_NOISE_results_update_frequency={}_Safe_Threshold={}.png'.format(update_frequency,
                                                                                        Safe_Threshold))
    plt.close()
    plt.plot(noise, self_collision, 'b*-', label='self collision')
    plt.plot(noise, obs_collision, 'b^-', label = 'osbtacle collision')
    plt.title('update_frequency={}_Safe_Threshold={}'.format(update_frequency, Safe_Threshold))
    plt.xlabel('DETECT_NOISE')
    plt.ylabel('collision')
    plt.savefig('Figures/DETECT_NOISE/collision/'
                'DETECT_NOISE_results_update_frequency={}_Safe_Threshold={}.png'.format(update_frequency,
                                                                                        Safe_Threshold))
    plt.close()
    plt.plot(noise, reachends, 'g^-', label='failed to reachend')
    plt.title('update_frequency={}_Safe_Threshold={}'.format(update_frequency, Safe_Threshold))
    plt.xlabel('DETECT_NOISE')
    plt.ylabel('failed to reachend')
    plt.savefig('Figures/DETECT_NOISE/reachend/'
                'DETECT_NOISE_results_update_frequency={}_Safe_Threshold={}.png'.format(update_frequency,
                                                                                        Safe_Threshold))
    plt.close()

def plot_update_results(DETECT_NOISE = 0.02, Safe_Threshold = 1.1):
    update_frequency = []
    path_length = []
    self_collision = []
    obs_collision = []
    faileds = []
    reachends = []
    for i in DF.index:
        # print(DF.loc[i, 'update_frequency'])
        if DF.loc[i, 'DETECT_NOISE'] == DETECT_NOISE:
            # print(DF.loc[i, 'Safe_Threshold'])
            if DF.loc[i, 'Safe_Threshold'] == Safe_Threshold:
                x = DF.loc[i, 'update_frequency']
                y1 = DF.loc[i, 'path_length']
                y2 = DF.loc[i, 'self_collision']
                y3 = DF.loc[i, 'obstacles_collision']
                y4 = DF.loc[i, 'failed']
                y5 = DF.loc[i, 'reachend']
                reachends.append(y5)
                faileds.append(y4)
                update_frequency.append(x)
                path_length.append(y1)
                self_collision.append(y2)
                obs_collision.append(y3)
    update_frequency = np.array(update_frequency)
    path_length = np.array(path_length)
    self_collision = np.array(self_collision)
    obs_collision = np.array(obs_collision)
    plt.plot(update_frequency, path_length, 'ro-', label='Distance')
    plt.plot(update_frequency, self_collision, 'b*-', label='self collision')
    plt.plot(update_frequency, obs_collision, 'b^-', label = 'osbtacle collision')
    # plt.plot(update_frequency, faileds, 'g^-', label = 'failed')
    plt.plot(update_frequency, reachends, 'g^-', label='failed to reachend')
    plt.title('DETECT_NOISE={}_Safe_Threshold={}'.format(DETECT_NOISE, Safe_Threshold))
    plt.xlabel('update_frequency')
    plt.ylabel('Distance')
    plt.legend()
    plt.savefig('Figures/update_frequency/'
                'update_frequency_results_DETECT_NOISE={}_Safe_Threshold={}.png'.format(DETECT_NOISE, Safe_Threshold))
    plt.close()
    plt.plot(update_frequency, path_length, 'ro-', label='Distance')
    plt.title('DETECT_NOISE={}_Safe_Threshold={}'.format(DETECT_NOISE, Safe_Threshold))
    plt.xlabel('update_frequency')
    plt.ylabel('Distance')
    plt.savefig('Figures/update_frequency/Distance/'
                'update_frequency_results_DETECT_NOISE={}_Safe_Threshold={}.png'.format(DETECT_NOISE, Safe_Threshold))
    plt.close()
    plt.plot(update_frequency, self_collision, 'b*-', label='self collision')
    plt.plot(update_frequency, obs_collision, 'b^-', label='osbtacle collision')
    plt.title('DETECT_NOISE={}_Safe_Threshold={}'.format(DETECT_NOISE, Safe_Threshold))
    plt.xlabel('update_frequency')
    plt.ylabel('collision')
    plt.savefig('Figures/update_frequency/collision/'
                'update_frequency_results_DETECT_NOISE={}_Safe_Threshold={}.png'.format(DETECT_NOISE, Safe_Threshold))
    plt.close()
    plt.plot(update_frequency, reachends, 'g^-', label='failed to reachend')
    plt.title('DETECT_NOISE={}_Safe_Threshold={}'.format(DETECT_NOISE, Safe_Threshold))
    plt.xlabel('update_frequency')
    plt.ylabel('failed to reachend')
    plt.savefig('Figures/update_frequency/reachend/'
                'update_frequency_results_DETECT_NOISE={}_Safe_Threshold={}.png'.format(DETECT_NOISE, Safe_Threshold))
    plt.close()

def plot_safeThres_results(update_frequency = 1.0, DETECT_NOISE = 1.0):
    Safe_Threshold = []
    path_length = []
    self_collision = []
    obs_collision = []
    faileds = []
    reachends = []
    for i in DF.index:
        # print(DF.loc[i, 'update_frequency'])
        if DF.loc[i, 'update_frequency'] == update_frequency:
            # print(DF.loc[i, 'Safe_Threshold'])
            if DF.loc[i, 'DETECT_NOISE'] == DETECT_NOISE:
                x = DF.loc[i, 'Safe_Threshold']
                y1 = DF.loc[i, 'path_length']
                y2 = DF.loc[i, 'self_collision']
                y3 = DF.loc[i, 'obstacles_collision']
                y4 = DF.loc[i, 'failed']
                y5 = DF.loc[i, 'reachend']
                reachends.append(y5)
                faileds.append(y4)
                Safe_Threshold.append(x)
                path_length.append(y1)
                self_collision.append(y2)
                obs_collision.append(y3)
    Safe_Threshold = np.array(Safe_Threshold)
    path_length = np.array(path_length)
    self_collision = np.array(self_collision)
    obs_collision = np.array(obs_collision)
    plt.plot(Safe_Threshold, path_length, 'ro-', label='Distance')
    plt.plot(Safe_Threshold, self_collision, 'b*-', label='self collision')
    plt.plot(Safe_Threshold, obs_collision, 'b^-', label = 'osbtacle collision')
    # plt.plot(Safe_Threshold, faileds, 'g^-', label = 'failed')
    plt.plot(Safe_Threshold, reachends, 'g^-', label='failed to reachend')
    plt.title('DETECT_NOISE={}_update_frequency={}'.format(DETECT_NOISE, update_frequency))
    plt.xlabel('Safe_Threshold')
    plt.ylabel('Distance')
    plt.legend()
    plt.savefig('Figures/Safe_Threshold/'
                'Safe_Threshold_results_DETECT_NOISE={}_update_frequency={}.png'.format(DETECT_NOISE, update_frequency))
    plt.close()
    plt.plot(Safe_Threshold, path_length, 'ro-', label='Distance')
    plt.title('DETECT_NOISE={}_update_frequency={}'.format(DETECT_NOISE, update_frequency))
    plt.xlabel('Safe_Threshold')
    plt.ylabel('Distance')
    plt.savefig('Figures/Safe_Threshold/Distance/'
                'Safe_Threshold_results_DETECT_NOISE={}_update_frequency={}.png'.format(DETECT_NOISE, update_frequency))
    plt.close()
    plt.plot(Safe_Threshold, self_collision, 'b*-', label='self collision')
    plt.plot(Safe_Threshold, obs_collision, 'b^-', label='osbtacle collision')
    plt.title('DETECT_NOISE={}_update_frequency={}'.format(DETECT_NOISE, update_frequency))
    plt.xlabel('Safe_Threshold')
    plt.ylabel('collision')
    plt.savefig('Figures/Safe_Threshold/collision/'
                'Safe_Threshold_results_DETECT_NOISE={}_update_frequency={}.png'.format(DETECT_NOISE, update_frequency))
    plt.close()
    plt.plot(Safe_Threshold, reachends, 'g^-', label='failed to reachend')
    plt.title('DETECT_NOISE={}_update_frequency={}'.format(DETECT_NOISE, update_frequency))
    plt.xlabel('Safe_Threshold')
    plt.ylabel('failed to reachend')
    plt.savefig('Figures/Safe_Threshold/reachend/'
                'Safe_Threshold_results_DETECT_NOISE={}_update_frequency={}.png'.format(DETECT_NOISE, update_frequency))
    plt.close()

def Mkdir(path):
    try:
        os.mkdir(path)
    except:
        pass

if __name__ == '__main__':
    Mkdir('Figures')
    Mkdir('Figures/Safe_Threshold/')
    Mkdir('Figures/Safe_Threshold/Distance/')
    Mkdir('Figures/Safe_Threshold/collision/')
    Mkdir('Figures/Safe_Threshold/failed/')
    Mkdir('Figures/Safe_Threshold/reachend/')
    Mkdir('Figures/update_frequency/')
    Mkdir('Figures/update_frequency/Distance/')
    Mkdir('Figures/update_frequency/collision/')
    Mkdir('Figures/update_frequency/failed/')
    Mkdir('Figures/update_frequency/reachend/')
    Mkdir('Figures/DETECT_NOISE/')
    Mkdir('Figures/DETECT_NOISE/Distance/')
    Mkdir('Figures/DETECT_NOISE/collision/')
    Mkdir('Figures/DETECT_NOISE/failed/')
    Mkdir('Figures/DETECT_NOISE/reachend/')
    noises = DF['DETECT_NOISE'].unique()
    update_frequencys = DF['update_frequency'].unique()
    Safe_Thresholds = DF['Safe_Threshold'].unique()
    for noise in noises:
        for update_frequency in update_frequencys:
            plot_safeThres_results(DETECT_NOISE=noise, update_frequency=update_frequency)
    for Safe_Threshold in Safe_Thresholds:
        for update_frequency in update_frequencys:
            plot_noise_results(update_frequency=update_frequency, Safe_Threshold=Safe_Threshold)
    for noise in noises:
        for Safe_Threshold in Safe_Thresholds:
            plot_update_results(DETECT_NOISE=noise, Safe_Threshold=Safe_Threshold)
