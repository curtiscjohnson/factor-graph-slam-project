#!/usr/bin/env python3
import numpy as np
from hwSLAM import graph_slam_known, get_measurements
import matplotlib.pyplot as plt
import time

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
    hw_odometry = hw_odometry[:,np.r_[1,0,2:4]]

    # hw_odometry = np.load("velocity_odometry_data.npy")

    m = hw_odometry.shape[0]

    # make length of hw_odometry match the length of measurements
    hw_odometry = hw_odometry[np.linspace(0, m-1, num=n, dtype=int), :]

    # fig2, axs = plt.subplots(1, 2)
    # axs[0].plot(hw_odometry[:, 1], hw_odometry[:, 2], label='robot position', c='k')
    # axs[1].plot(hw_odometry[:, 3], label='robot base theta')
    # axs[0].axis('equal')
    # axs[0].legend()
    # axs[1].legend()
    # plt.show()

    mu = [0, 0, 0]

    graph_slam = graph_slam_known(mu, prior_sigmas=np.array([0,0,0]),
                                      odo_sigmas=np.array([10, 10, 0.1]), 
                                      loose_sigma=10, 
                                      meas_sigmas=np.array([10, .5]),
                                      minK=50,
                                      incK=50)

    for step in range(n):
        z = hw_measurements[step][1:] # get the measurements (without the timestep at the beginning)
        u = hw_odometry[step][1:] # get the odometry data (without the timestep at the beginning)
        
        mu = graph_slam.step(u,z)
        if step % 20 == 0:
            graph_slam.plot_step(mu)
            time.sleep(0.1)

    print("finished")
    


if __name__ == "__main__":
    run()
