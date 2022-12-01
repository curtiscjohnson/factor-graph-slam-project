from bagpy import bagreader
import pandas as pd
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from tqdm import tqdm


def rot(th):
    return np.array([[np.cos(th), -np.sin(th)],
                      [np.sin(th), np.cos(th)]])


class RobotViz:
    def __init__(self, ax: plt.axis):
        self.ax = ax
        boxPoints = self.get_box_points(np.zeros((2,)), 0)
        self.box, = ax.plot(boxPoints[:, 0], boxPoints[:, 1], c='k', lw=1.0, label='robot')
        wheelPoints = self.get_wheel_points(np.zeros((2,)), 0, np.zeros((4,)))
        self.wheels = LineCollection(wheelPoints, color=(1, 0, 0, 1))
        self.ax.add_collection(self.wheels)
        self.ax.set_xlim([-1, 19])
        self.ax.set_ylim([-10, 10])
        # self.ax.axis('equal')

    def get_box_points(self, p: np.ndarray, th: float):
        points = np.array([[-0.4, -0.56],
                           [0.4, -0.56],
                           [0.4, 0.56],
                           [-0.4, 0.56],
                           [-0.4, -0.56]])
        R = rot(th)
        points = points @ R.T + p
        return points

    def get_wheel_points(self, p: np.ndarray, th: float, wth: np.ndarray):
        wheelPointList = []

        R = rot(th)

        r0 = np.array([-0.4, -0.56])
        r1 = np.array([0.4, -0.56])
        r2 = np.array([-0.4, 0.56])
        r3 = np.array([0.4, 0.56])

        R0 = rot(wth[0])
        w0 = np.array([[-0.1, 0],
                       [0.1, 0]])
        w0 = (w0 @ R0.T + r0) @ R.T + p
        wheelPointList.append(w0)

        R1 = rot(wth[1])
        w1 = np.array([[-0.1, 0],
                       [0.1, 0]])
        w1 = (w1 @ R1.T + r1) @ R.T + p
        wheelPointList.append(w1)

        R2 = rot(wth[2])
        w2 = np.array([[-0.1, 0],
                       [0.1, 0]])
        w2 = (w2 @ R2.T + r2) @ R.T + p
        wheelPointList.append(w2)

        R3 = rot(wth[3])
        w3 = np.array([[-0.1, 0],
                       [0.1, 0]])
        w3 = (w3 @ R3.T + r3) @ R.T + p
        wheelPointList.append(w3)

        return wheelPointList

    def update(self, p, th, wth):
        boxPoints = self.get_box_points(p, th)
        self.box.set_data(boxPoints[:, 0], boxPoints[:, 1])
        wheelPoints = self.get_wheel_points(p, th, wth)
        self.wheels.set_segments(wheelPoints)
        plt.pause(0.001)


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
n = int(n / 10)
print(f"n = {n}")

# Robot physical parameters
radPerTick = 4 * np.pi / 180
wheelDiameter = 0.255
r0 = np.array([-0.4, -0.56])
r1 = np.array([0.4, -0.56])
r2 = np.array([-0.4, 0.56])
r3 = np.array([0.4, 0.56])
rs = np.vstack([r0, r1, r2, r3])

phiOffset = np.array([np.arctan2(r1[1] - r0[1], r1[0] - r0[0]),
                      np.arctan2(r2[1] - r1[1], r2[0] - r1[0]),
                      np.arctan2(r3[1] - r2[1], r3[0] - r2[0]),
                      np.arctan2(r0[1] - r3[1], r0[0] - r3[0])])

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

# n = 500
#
# wheelDistance = np.linspace(np.array([0, 0, 0, 0]), np.array([1, 1, 1, 1]), n)
#
# wheelRelativeTheta = np.linspace(np.array([0, 1, 0, 0]),
#                                  np.array([0, 0, 0, 0]),
#                                  n)

wheelDelta = np.zeros_like(wheelDistance)
for i in range(1, n):
    wheelDelta[i] = wheelDistance[i] - wheelDistance[i - 1]

# Now the fun part.
# Robot state represents the robot's position as [x y theta]
robotState = np.zeros((n, 3))

# wheelPos holds the position of each wheel in the world frame. WheelTheta holds the orientation
# of each wheel in the world frame
wheelPos = np.zeros((n, 4 * 2))
wheelTheta = np.zeros((n, 4))

wheelPos[0] = np.ravel(rs)
wheelTheta[0] = wheelRelativeTheta[0] + robotState[0, 2]


def robot_state_to_wheel_pos(r: np.ndarray):
    thr = r[2]
    R = rot(thr)
    wpos = np.zeros((8,))
    for i in range(4):
        wpos[2 * i:2 * i + 2] = R @ rs[i] + r[0:2]
    return wpos


def robot_state_to_prediction(r: np.ndarray):
    thr = r[2]
    bhat_pos = robot_state_to_wheel_pos(r)
    bhat_phi = np.zeros((4,))
    for i in range(4):
        bhat_phi[i] = phiOffset[i] + thr
    return np.hstack([bhat_pos, bhat_phi])


def wheel_state_to_prediction(wp):
    b_pos = np.copy(wp)
    b_phi = np.zeros((4,))
    for i in range(3):
        v = wp[2 * i + 2:2 * i + 4] - wp[2 * i:2 * i + 2]
        b_phi[i] = np.arctan2(v[1], v[0])
    v = wp[0:2] - wp[6:8]
    b_phi[3] = np.arctan2(v[1], v[0])
    return np.hstack([b_pos, b_phi])


def least_squares_error(r, *args):
    wp = args[0]
    bhat = robot_state_to_prediction(r)
    b = wheel_state_to_prediction(wp)
    return b - bhat


def propagate_wheel_pos(wp_prev, wth, wdelta):
    wphat = np.zeros((8,))
    for j, th in enumerate(wth):
        v = np.array([np.cos(th), np.sin(th)])
        wphat[2 * j:2 * j + 2] = wp_prev[2 * j:2 * j + 2] + wdelta[j] * v
    return wphat


fig, ax = plt.subplots()
viz = RobotViz(ax)

for i in tqdm(range(1, n)):
    wheelTheta[i] = wheelRelativeTheta[i] + robotState[i - 1, 2]
    wphat = propagate_wheel_pos(wheelPos[i - 1], wheelTheta[i], wheelDelta[i])
    sol = least_squares(least_squares_error, x0=robotState[i - 1], args=(wphat,))
    robotState[i] = sol.x
    wheelPos[i] = robot_state_to_wheel_pos(robotState[i])

    if i % 10 == 0:
        viz.update(robotState[i, 0:2], robotState[i, 2], wheelTheta[i])

fig2, axs = plt.subplots(1, 3)
axs[0].plot(robotState[:, 0], robotState[:, 1], label='robot position', c='k')
axs[0].plot(wheelPos[:, 0], wheelPos[:, 1], label='wheel 1', c='b')
axs[0].plot(wheelPos[:, 2], wheelPos[:, 3], label='wheel 1', c='b')
axs[0].plot(wheelPos[:, 4], wheelPos[:, 5], label='wheel 1', c='b')
axs[0].plot(wheelPos[:, 6], wheelPos[:, 7], label='wheel 1', c='b')
axs[2].plot(robotState[:, 2], label='robot base label')
axs[0].axis('equal')
axs[0].legend()
axs[1].legend()
axs[2].legend()


plt.show()
