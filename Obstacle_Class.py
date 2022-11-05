import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import pyplot as plt

class Obstacle():
    def __init__(self, position_x, position_y, length, width, height):
        self.x = position_x
        self.y = position_y
        self.length = length
        self.width = width
        self.height = height

if __name__ == "__main__":
    obs = []
    for i in range(50):
        obs.append(Obstacle(3 + 3 * np.random.randn(), 3 + 3 * np.random.randn(), 0.1, 0.1, abs(np.random.randn())))
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    X = np.array([ ob.x for ob in obs])
    Y = np.array([ ob.y for ob in obs])
    height = np.array([ ob.height for ob in obs])
    bottom=np.zeros_like(X)#设置柱状图的底端位值
    height = height
    length=np.array([ ob.length for ob in obs])
    width=np.array([ ob.width for ob in obs])#每一个柱子的长和宽
    #绘图设置
    ax.bar3d(X, Y, bottom, length, width, height, shade=True)#
    plt.show()