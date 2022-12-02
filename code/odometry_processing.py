from bagpy import bagreader
import pandas as pd
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from tqdm import tqdm


# Robot physical parameters
radPerTick = 4 * np.pi / 180
wheelDiameter = 0.255

"""
            0.495
          |-----|
1. ______________ 0.            _
   |            |        ^ X    |  
   |            |        |      | 0.47
   |      x     |   Z <--|      |
   |            |               -
   |            |
   --------------
3.                2.
"""

r0 = np.array([0.47, -0.495])
r1 = np.array([0.47, 0.495])
r2 = np.array([-0.47, -0.495])
r3 = np.array([-0.47, 0.495])
rs = np.vstack([r0, r1, r2, r3, r0])


# Robot Visualization
class RobotViz:
    def __init__(self, ax: plt.axis):
        self.ax = ax
        boxPoints = self.get_box_points(np.zeros((4,)))
        self.box, = ax.plot(boxPoints[:, 0], boxPoints[:, 1], c='k', lw=1.0, label='robot')
        wheelPoints = self.get_wheel_points(np.zeros((4,)), np.zeros((4,)))
        self.wheels = LineCollection(wheelPoints, color=(1, 0, 0, 1))
        self.ax.add_collection(self.wheels)
        self.ax.axis('square')
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])

    @staticmethod
    def get_box_points(r):
        p = r[[0, 2]]
        th = r[3]
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
        p = r[[0, 2]]
        th = r[3]
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
        plt.pause(0.001)


# Utility Functions
def rot(th):
    return np.array([[np.cos(th), np.sin(th)],
                     [-np.sin(th), np.cos(th)]])


def wrap_angle(th):
    if isinstance(th, np.ndarray):
        ar = np.zeros_like(th)
        for k, t in enumerate(th):
            ar[k] = wrap_angle(t)
        return ar
    else:
        while th > np.pi:
            th = th - 2 * np.pi
        while th < -np.pi:
            th = th + 2 * np.pi
        return th


# Kinematics Equations
def robot_state_to_wheel_pos(r: np.ndarray):
    p = r[[0, 2]]
    thr = r[3]
    R = rot(thr)
    wpos = np.zeros((4, 2))
    for i in range(4):
        wpos[i] = rs[i] @ R.T + p
    return wpos


def least_squares_error(r, *args):
    wp = args[0]
    bhat = robot_state_to_wheel_pos(r)
    error = np.ravel(bhat - wp)
    return error


def propagate_wheel_pos(wp_prev, wth, wdelta):
    wphat = np.zeros((4, 2))
    for i, th in enumerate(wth):
        th = wth[i]
        v = np.array([np.cos(th), -np.sin(th)])
        wphat[i] = wp_prev[i] + v * wdelta[i]
    return wphat


# Load bag file and encoder data
b = bagreader('./slam_data_1.bag')
encoder_csv = b.message_by_topic('/mobile_base_0/encoders')
encoder_data = pd.read_csv(encoder_csv)
encoder_data['Time'] = encoder_data['Time'] - encoder_data['Time'][0]
encoder_data['segway.segway_0'] = encoder_data['segway.segway_0'] - encoder_data['segway.segway_0'][0]
encoder_data['segway.segway_1'] = encoder_data['segway.segway_1'] - encoder_data['segway.segway_1'][0]
encoder_data['segway.segway_2'] = encoder_data['segway.segway_2'] - encoder_data['segway.segway_2'][0]
encoder_data['segway.segway_3'] = encoder_data['segway.segway_3'] - encoder_data['segway.segway_3'][0]

n = encoder_data['Time'].shape[0]
n = int(n)
print(f"n = {n}")


# Convert encoder data to distance traveled per wheel data
# wheelDelta is an array of distance values, starting at zero and accounting for the total distance traveled by each
# wheel. Also merge the separate castor values into one array for convenience

wheelDistance = np.vstack([encoder_data['segway.segway_0'],
                           encoder_data['segway.segway_1'],
                           encoder_data['segway.segway_2'],
                           encoder_data['segway.segway_3']]).T * radPerTick * wheelDiameter / 2

wheelRelativeTheta = np.vstack([encoder_data['castor.castor_0'],
                                encoder_data['castor.castor_1'],
                                encoder_data['castor.castor_2'],
                                encoder_data['castor.castor_3']]).T

# n = 1000
#
# wheelDistance = np.linspace(np.array([0, 0, 0, 0]), np.array([1, 1, 1, 1]), n)
#
# th0 = np.arctan2(r0[1], r0[0]) #- np.pi / 2
# th1 = np.arctan2(r1[1], r1[0]) #- np.pi / 2
# th2 = np.arctan2(r2[1], r2[0]) #- np.pi / 2
# th3 = np.arctan2(r3[1], r3[0]) #- np.pi / 2
#
# wheelRelativeTheta = np.linspace(np.array([0, 0, 0, 0]),
#                                  np.array([0, 0, 0, 0]),
#                                  n)

wheelDelta = np.zeros_like(wheelDistance)
for i in range(1, n):
    wheelDelta[i] = wheelDistance[i] - wheelDistance[i - 1]

# Now the fun part.
# Robot state represents the robot's position as [x y theta]
robotState = np.zeros((n, 4))
robotState[0] = np.array([0, 0, 0, 0])  # [x y z theta]

# wheelPos holds the position of each wheel in the world frame. WheelTheta holds the orientation
# of each wheel in the world frame
wheelPos = np.zeros((n, 4, 2))
wheelTheta = np.zeros((n, 4))

wheelPos[0] = robot_state_to_wheel_pos(robotState[0])
wheelTheta[0] = wheelRelativeTheta[0] + robotState[0, 3]

fig, ax = plt.subplots()
viz = RobotViz(ax)

for i in tqdm(range(1, n)):
    # if i == 328:
    #     print('help')
    wheelTheta[i] = wrap_angle(wheelRelativeTheta[i] + robotState[i - 1, 3])
    wphat = propagate_wheel_pos(wheelPos[i - 1], wheelTheta[i], wheelDelta[i])

    # Least Squares Error Solution
    sol = least_squares(least_squares_error, x0=robotState[i - 1], args=(wphat,))
    robotState[i] = sol.x
    robotState[i, 1] = 0

    # Wrap angles
    robotState[i, 3] = wrap_angle(robotState[i, 3])

    # Set wheel position
    wheelPos[i] = robot_state_to_wheel_pos(robotState[i])

    # Animate every x steps
    if i % 10000000 == 0:
        viz.update(robotState[i], wheelRelativeTheta[i])

np.save('robot_state_as_np_array-position_odometry', robotState)

fig2, axs = plt.subplots(1, 2)
axs[0].plot(robotState[:, 0], robotState[:, 2], label='robot position', c='k')
# axs[0].plot(wheelPos[:, 0], wheelPos[:, 1], label='wheel 1', c='b')
# axs[0].plot(wheelPos[:, 2], wheelPos[:, 3], label='wheel 2', c='b')
# axs[0].plot(wheelPos[:, 4], wheelPos[:, 5], label='wheel 3', c='b')
# axs[0].plot(wheelPos[:, 6], wheelPos[:, 7], label='wheel 4', c='b')
axs[1].plot(robotState[:, 3], label='robot base theta')
axs[0].axis('equal')
axs[0].legend()
axs[1].legend()
# axs[2].legend()


plt.show()
