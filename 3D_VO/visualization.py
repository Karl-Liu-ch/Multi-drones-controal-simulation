from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation

def visualization(robots_states, robots_radius, obstacles_states, obs_radius, obs_heights, NUMBER_OF_TIMESTEPS):
    fig = plt.figure()
    ax = plt.axes(projection="3d")

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_zlim(0, 10)

    x0 = [robots_radius[i] * np.outer(np.cos(u), np.sin(v)) for i in range(len(robots_states))]
    y0 = [robots_radius[i] * np.outer(np.sin(u), np.sin(v)) for i in range(len(robots_states))]
    z0 = [robots_radius[i] * np.outer(np.ones(np.size(u)), np.cos(v)) for i in range(len(robots_states))]
    u = np.linspace(0, 2 * np.pi, 50)  # 把圆分按角度为50等分
    h = np.linspace(0, 1, 20)  # 把高度1均分为20份
    x = [obs_radius[i] * np.outer(np.sin(u), np.ones(len(h))) for i in range(len(obstacles_states))]
    y = [obs_radius[i] * np.outer(np.cos(u), np.ones(len(h))) for i in range(len(obstacles_states))]
    z = [obs_heights[i] * np.outer(np.ones(len(u)), h) for i in range(len(obstacles_states))]
    surface_color = "tab:blue"
    def init():
        for i in range(len(robots_states)):
            ax.plot_surface(x0[i] + robots_states[i][0, 0], y0[i] + robots_states[i][1, 0],
                            z0[i] + robots_states[i][2, 0], color=surface_color)
        for j in range(len(obstacles_states)):
            ax.plot_surface(x[j] + obstacles_states[j][0, 0], y[j] + obstacles_states[j][1, 0],
                            z[j] + obstacles_states[j][2, 0], cmap=plt.get_cmap('Greens'))
        return fig,

    # def plot_round_bar(obstacles):
    #     u = np.linspace(0, 2 * np.pi, 50)  # 把圆分按角度为50等分
    #     h = np.linspace(0, 1, 20)  # 把高度1均分为20份
    #     for ob in obstacles:
    #         x = ob.radius * np.outer(np.sin(u), np.ones(len(h))) + ob.position[0] # x值重复20次
    #         y = ob.radius * np.outer(np.cos(u), np.ones(len(h))) + ob.position[1]  # y值重复20次
    #         z = ob.height * np.outer(np.ones(len(u)), h)  # x，y 对应的高度
    #         ax.plot_surface(x, y, z, cmap=plt.get_cmap('Greens'))
    # plot_round_bar(obstacles)

    def animate(t):
        # remove previous collections
        ax.collections.clear()
        # ax.bar3d(X, Y, bottom, length, width, height, shade=True, color="green")
        # plot_round_bar(obstacles)
        for j in range(len(robots_states)):
            robot_state = robots_states[j]
            position = robot_state[:3, t]
            px, py, pz = position[0], position[1], position[2]
            ax.plot_surface(x0[j] + px, y0[j] + py, z0[j] + pz, color=surface_color)
        for k in range(len(obstacles_states)):
            ob_state = obstacles_states[k]
            position = ob_state[:3, t]
            px, py, pz = position[0], position[1], position[2]
            ax.plot_surface(x[k] + px, y[k] + py, z[k] + pz, cmap=plt.get_cmap('Greens'))
        return fig,

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=NUMBER_OF_TIMESTEPS, interval=300)
    plt.show()

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits import mplot3d
    from matplotlib import cm
    from matplotlib import animation
    import pandas as pd


    fig = plt.figure(facecolor='black')
    ax = plt.axes(projection = "3d")

    u = np.linspace(0, 2*np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    r = 4

    ax.set_xlim(0, 60)
    ax.set_ylim(0, 60)
    ax.set_zlim(0, 60)

    x0 = [r * np.outer(np.cos(u), np.sin(v)) + 10 + i * 10 for i in range(2)]
    y0 = [r * np.outer(np.sin(u), np.sin(v)) + 10 + i * 10 for i in range(2)]
    z0 = [r * np.outer(np.ones(np.size(u)), np.cos(v)) + 30 + i * 30 for i in range(2)]
    surface_color = "tab:blue"

    def init():
        for i in range(2):
            ax.plot_surface(x0[i], y0[i], z0[i], color=surface_color)
        return fig,

    def animate(i):
        # remove previous collections
        ax.collections.clear()
        # add the new sphere
        for j in range(2):
            ax.plot_surface(x0[j] + i, y0[j] + i, z0[j] + i, color=surface_color)
        return fig,

    ani = animation.FuncAnimation(fig, animate, init_func = init, frames = 90, interval = 300)

    plt.show()