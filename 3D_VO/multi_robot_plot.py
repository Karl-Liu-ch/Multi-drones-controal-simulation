import sys
sys.path.append('../')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np

def plot_robot_and_obstacles(robots, obstacles, robots_radius, obs_radius, num_steps, sim_time, filename):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 20), ylim=(0, 20))
    ax.set_aspect('equal')
    ax.grid()
    lines = [] #, = ax.plot([], [], '--r')

    robot_list = []
    for i in range(len(robots)):
        robot_cur = Circle((robots[i][0, 0], robots[i][1, 0]),
                         robots_radius[i], facecolor='green', edgecolor='black')
        robot_list.append(robot_cur)
        line, = ax.plot([], [], '--r')
        lines.append(line)
    #robot_patch = Circle((robot[0, 0], robot[1, 0]),
    #                     robot_radius, facecolor='green', edgecolor='black')

    obstacle_list = []
    for j in range(len(obstacles)):
        obstacle_cur = Circle((obstacles[j][0, 0], obstacles[j][1, 0]),
                           obs_radius[j], facecolor='blue', edgecolor='black')
        obstacle_list.append(obstacle_cur)
        line, = ax.plot([], [], '--r')
        lines.append(line)

    # obstacle_list = []
    # for obstacle in range(np.shape(obstacles)[2]):
    #     obstacle = Circle((0, 0), robot_radius,
    #                       facecolor='aqua', edgecolor='black')
    #     obstacle_list.append(obstacle)

    def init():
        #ax.add_patch(robot_patch)
        for i in range(len(robot_list)):
            ax.add_patch(robot_list[i])
            lines[i].set_data([], [])
        for obstacle in obstacle_list:
            ax.add_patch(obstacle)
        #line.set_data([], [])
        return robot_list + lines + obstacle_list
        #return [robot_patch] + [line] + obstacle_list

    def animate(i):
        for j in range(len(robot_list)):
            robot_list[j].center = (robots[j][0, i], robots[j][1, i])
            lines[j].set_data(robots[j][0, :i], robots[j][1, :i])
        #robot_patch.center = (robot[0, i], robot[1, i])
        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[j][0, i], obstacles[j][1, i])
        #line.set_data(robot[0, :i], robot[1, :i])
        return robot_list, lines, obstacle_list
        #return [robot_patch] + [line] + obstacle_list

    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step)

    # Save animation
    if not filename:
        return

    ani = animation.FuncAnimation(
        fig, animate, np.arange(1, num_steps), interval=200,
        blit=True, init_func=init)

    ani.save(filename, "ffmpeg", fps=30)


def plot_robot(robot, timestep, radius=1, is_obstacle=False):
    if robot is None:
        return
    center = robot[:2, timestep]
    x = center[0]
    y = center[1]
    if is_obstacle:
        circle = plt.Circle((x, y), radius, color='aqua', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], '--r',)
    else:
        circle = plt.Circle((x, y), radius, color='green', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], 'blue')

    plt.gcf().gca().add_artist(circle)
