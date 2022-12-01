#!/usr/bin/env python3
import numpy as np
from simSLAM import simulation
from hwSLAM import graph_slam_known, get_measurements, hw_plot

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
    alphas = np.array([1, 1, np.pi])**2 # These have to do with the error in our motion controls
    betas = np.array([0.05, 0.01, 0.1]) # Error in observations

                # np.array([t, x, y, theta])
    hw_odometry = np.load("velocity_odometry_data.npy")

    hw_measurements = get_measurements()

    mu = [0, 0, 0]
    cov = 1e-03*np.eye(3) # start covariance low but non-zero

    graph_slam = graph_slam_known(mu, alphas, betas)
    plotter = hw_plot()

    # Find the last time recorded for while loop condition
    last_time = hw_measurements[-1][0] if hw_measurements[-1][0] > hw_odometry[-1][0] else hw_odometry[-1][0]
    dt = 0.1
    curr_t = 0
    u_ctr = 0
    z_ctr = 0

    while curr_t < last_time:
        if (hw_measurements[z_ctr][0] > curr_t):
            # Use the range/bearing measurement
            z = hw_measurements[z_ctr][1:] # get the measurements (without the timestep at the beginning)
        else:
            z = []
            z_ctr += 1

        if (hw_odometry[u_ctr][0] > curr_t):
            # use the odometry measurement
            u = hw_odometry[u_ctr][1:] # get the odometry data (without the timestep at the beginning)
        else:
            u = []
            u_ctr += 1

        if len(z) + len(u) != 0:
            out = graph_slam.step(u,z)
            # plotter.plot_step(mu, cov)

        # Increment time
        curr_t += dt

    print(mu, cov)


if __name__ == "__main__":
    run()
