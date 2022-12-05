import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from helpers import rot


r0 = np.array([0.47, -0.495])
r1 = np.array([0.47, 0.495])
r2 = np.array([-0.47, -0.495])
r3 = np.array([-0.47, 0.495])
rs = np.vstack([r0, r1, r2, r3, r0])

class RobotViz:
    def __init__(self, ax: plt.axis, r=np.zeros((4,))):
        self.ax = ax
        boxPoints = self.get_box_points(np.zeros((4,)))
        self.box, = ax.plot(boxPoints[:, 0], boxPoints[:, 1], c='k', lw=1.0, label='robot')
        wheelPoints = self.get_wheel_points(np.zeros((4,)), np.zeros((4,)))
        self.wheels = LineCollection(wheelPoints, color=(1, 0, 0, 1))
        self.ax.add_collection(self.wheels)
        self.ax.axis('square')
        self.xs = [r[0]]
        self.zs = [r[2]]
        self.path, = self.ax.plot(self.xs, self.zs, label='dead reckoning path', c=[0.7, 0.5, 0.1, 0.7])

    @staticmethod
    def get_box_points(r):
        p = r[0:2]
        th = -r[2]
        R = rot(th)
        points = np.array([[0.495, -0.4],
                           [0.495, 0.4],
                           [-0.495, 0.4],
                           [-0.495, -0.4],
                           [0.495, -0.4]])
        points = points @ R.T + p
        return points

    @staticmethod
    def get_wheel_points(r=np.zeros((4,)), wth=np.zeros((4,))):
        p = r[0:2]
        th = -r[2]
        wheelPointList = []

        wheelShape = np.array([[-0.15, 0],
                               [0.15, 0],
                               [0.1, -0.05],
                               [0.1, 0.05],
                               [0.15, 0]])

        # rotation about y-axis
        R = rot(th)

        R0 = rot(wth[0])
        w0 = (wheelShape @ R0.T + r0) @ R.T + p
        wheelPointList.append(w0)

        R1 = rot(wth[1])
        w1 = (wheelShape @ R1.T + r1) @ R.T + p
        wheelPointList.append(w1)

        R2 = rot(wth[2])
        w2 = (wheelShape @ R2.T + r2) @ R.T + p
        wheelPointList.append(w2)

        R3 = rot(wth[3])
        w3 = (wheelShape @ R3.T + r3) @ R.T + p
        wheelPointList.append(w3)

        return wheelPointList

    def update(self, r, wth):
        boxPoints = self.get_box_points(r)
        self.box.set_data(boxPoints[:, 0], boxPoints[:, 1])
        wheelPoints = self.get_wheel_points(r, wth)
        self.wheels.set_segments(wheelPoints)
        self.xs.append(r[0])
        self.zs.append(r[1])
        self.path.set_data(self.xs, self.zs)
        xmin, xmax, zmin, zmax = np.min(self.xs), np.max(self.xs), np.min(self.zs), np.max(self.zs)
        self.ax.set_xlim([xmin - 3, xmax + 3])
        self.ax.set_ylim([zmin - 3, zmax + 3])
        plt.pause(0.001)