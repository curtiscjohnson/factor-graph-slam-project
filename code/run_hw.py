#!/usr/bin/env python3
import numpy as np
from hwSLAM import graph_slam_known, get_measurements
import matplotlib.pyplot as plt

def run():
    """" 
    General Steps:
        - Get data from txt file to list or numpy array
        - set general parameters (initialMu, initialSigma, alphas, betas, etc.)
        - For each time step recovered from the data:
            - Get measurement and odometry data
            - Give measurement and odometry data to SLAM algorithm to receive mean and Sigma
            - Plot the step (if we want to do live plotting)
        - Display final path with obstacles marked
    """
    # System Noise parameters
    alphas = np.array([0,0,0]) # These have to do with the error in our motion controls
    betas = np.array([0.05, 0.1, 0.1]) # Error in observations

    hw_measurements = get_measurements()
    n = len(hw_measurements)

                # np.array([t, x, y, theta])
    hw_odometry = np.load("position_odometry_data.npy")
    # hw_odometry = np.load("velocity_odometry_data.npy")

    m = hw_odometry.shape[0]

    # make length of hw_odometry match the length of measurements
    hw_odometry = hw_odometry[np.linspace(0, m-1, num=n, dtype=int), :]

    # fig2, axs = plt.subplots(1, 2)
    # axs[0].plot(hw_odometry[:, 0], hw_odometry[:, 2], label='robot position', c='k')
    # axs[1].plot(hw_odometry[:, 3], label='robot base theta')
    # axs[0].axis('equal')
    # axs[0].legend()
    # axs[1].legend()
    # plt.show()

    mu = [0, 0, 0]
    cov = 1e-03*np.eye(3) # initial covariance low but non-zero

    graph_slam = graph_slam_known(mu, alphas, betas)

    for step in range(n):
        z = hw_measurements[step][1:] # get the measurements (without the timestep at the beginning)
        u = hw_odometry[step][1:] # get the odometry data (without the timestep at the beginning)

        result = graph_slam.step(u,z)
        if step % 20 == 0:
            graph_slam.plot_step(result)

    
    graph_slam.finalize()
    print("finished")
    


if __name__ == "__main__":
    run()
