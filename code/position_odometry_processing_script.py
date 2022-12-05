from bagpy import bagreader
import pandas as pd
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from tqdm import tqdm
from helpers import rot, wrap_angle, compose
from robotvizualization import RobotViz

# Use this one for VS Code
# relative_file_path = './code/'

# Ignore this, on John's computer this is the path to data
relative_file_path = ''


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
wpos = np.vstack([r0, r1, r2, r3])





# Kinematics Equations
def robot_state_to_wheel_pos(r: np.ndarray):
    p = r[0:2]
    thr = r[2]
    R = rot(thr)
    w = wpos @ R.T + p
    return w


def least_squares_error(r, *args):
    wp = args[0]
    bhat = robot_state_to_wheel_pos(r)
    error = np.ravel(bhat - wp)
    return error


def propagate_wheel_pos(wth, wdelta):
    wphat = np.zeros((4, 2))
    for i, th in enumerate(wth):
        th = wth[i]
        v = np.array([np.cos(th), -np.sin(th)])  # -sin is because wheel relative angle is measured along the y-axis,
        # which is pointing down
        wphat[i] = wpos[i] + v * wdelta[i]
    return wphat


# Load bag file and encoder data
b = bagreader(relative_file_path + 'slam_data_1.bag')
encoder_csv = b.message_by_topic('/mobile_base_0/encoders')
encoder_data = pd.read_csv(encoder_csv)

dN = 5

encoder_data = encoder_data.iloc[::dN]

encoder_data['Time'] = encoder_data['Time'] - encoder_data['Time'][0]
encoder_data['segway.segway_0'] = encoder_data['segway.segway_0'] - encoder_data['segway.segway_0'][0]
encoder_data['segway.segway_1'] = encoder_data['segway.segway_1'] - encoder_data['segway.segway_1'][0]
encoder_data['segway.segway_2'] = encoder_data['segway.segway_2'] - encoder_data['segway.segway_2'][0]
encoder_data['segway.segway_3'] = encoder_data['segway.segway_3'] - encoder_data['segway.segway_3'][0]

n = encoder_data['Time'].shape[0]
n = int(n / 1)
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

wheelDelta = np.zeros_like(wheelDistance)
for i in range(1, n):
    wheelDelta[i] = wheelDistance[i] - wheelDistance[i - 1]

# Now the fun part.
# Robot state represents the robot's position as [x y theta]
relativePose = np.zeros((n, 3))

# wheelPos holds the position of each wheel in the world frame. WheelTheta holds the orientation
# of each wheel in the world frame

fig, ax = plt.subplots()
viz = RobotViz(ax)

r = np.array([0, 0, 0])

for i in tqdm(range(1, n)):

    wphat = propagate_wheel_pos(wheelRelativeTheta[i], wheelDelta[i])

    # Least Squares Error Solution
    sol = least_squares(least_squares_error, x0=np.hstack([np.mean(wphat, axis=0), 0]), args=(wphat,))
    relativePose[i] = sol.x

    # Wrap angles
    relativePose[i, 2] = wrap_angle(relativePose[i, 2])

    r = compose(r, relativePose[i])

    # Animate every x steps
    if i % 10 == 0:
        # r = np.array([0, 0, np.pi / 4])
        viz.update(r, wheelRelativeTheta[i])

odometry_data = np.hstack([np.asarray(encoder_data['Time'])[0:n, None], relativePose])
np.save(relative_file_path + 'position_odometry_data.npy', odometry_data)

plt.show()
