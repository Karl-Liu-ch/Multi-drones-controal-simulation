import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import pyplot as plt

class Obstacle():
    def __init__(self, position_x, position_y, position_z, length, width, height):
        self.x = position_x
        self.y = position_y
        self.z = position_z
        self.length = length
        self.width = width
        self.height = height


# fig = plt.figure(figsize=(50, 50))
# ax = p3.Axes3D(fig)
# X=np.arange(0, 5, step=1)#X轴的坐标
# Y=np.arange(0, 9, step=1)#Y轴的坐标
# Z=np.zeros(shape=(5, 9))
# for i in range(5):
#     for j in range(9):
#         Z[i, j]=i+j
# xx, yy=np.meshgrid(X, Y)#网格化坐标
# X, Y=xx.ravel(), yy.ravel()#矩阵扁平化
# bottom=np.zeros_like(X)#设置柱状图的底端位值
# Z=Z.ravel()#扁平化矩阵
# width=height=0.1#每一个柱子的长和宽
# #绘图设置
# ax=fig.gca(projection='3d')#三维坐标轴
# ax.bar3d(X, Y, bottom, width, height, Z, shade=True)#
# plt.show()