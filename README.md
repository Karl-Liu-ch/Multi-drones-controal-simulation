# Multi-drones-controal-simulation
Project of drone cluster control
## How to run
The simulator of drone cluster can be run directly by running 3D_VO/UAV_size_changeable_3d.py. 
Users can also customize the starting position, destination position and obstacle position of the drone by modifying the drone cluster parameters and obstacle parameters in the 3D_VO/UAV_size_changeable_3d.py file, simulation function. 
Besides, there are several parameters can also be manipulated, such as SIM_TIME, 
TIMESTEP, DETECT_NOISE, update_frequency and Safe_Threshold. The simulated result will be saved in simulation_results folder, which includes all information of drone cluster. 

To visualize the movement of drone cluster, just load the results by function load_uavs_obs(PATH=PATH) and run visualization function. 

To see the results of collision, traveled distance and whether drones reaching their destination, just run 3D_VO/Results_analysis.py and all figure will be saved in Figures folder. 

By running 2d_traverse.py, a 2D discrete simulation will be shown. 

We also tried deep reinforcement learning method to make drones avoid collision. However since it took too much time training agent and difficulty in manipulating the hyper parameters, we cannot get a well performed agent. But you can also give it a try by running DRL/PPO_continuous_main.py. 