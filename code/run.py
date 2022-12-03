#!/usr/bin/env python3
import numpy as np
import argparse
from simSLAM import simulation
from simSLAM import ekf_slam
from simSLAM import graph_slam_known

def run():

    numSteps = 200
    # System Noise parameters
    alphas = np.array([0.05, 0.001, 0.05, 0.01])**2 # These have to do with the error in our motion controls
    betas = np.array([10, 10*np.pi/180]) # Error in observations
    soccer_bot = simulation(numSteps=numSteps,alphas=alphas,betas=betas)

    mu = soccer_bot.get_initialStateMean()
    cov= 1e-03*np.eye(3) # start covariance low but non-zero
    ekf_SLAM_alg = ekf_slam(mu,cov,alphas=alphas,betas=betas)

    graph_alg = graph_slam = graph_slam_known(mu, prior_sigmas=np.array([0,0,0]),
                                            odo_sigmas=np.array([10, 10, 0.1]), 
                                            loose_sigma=10, 
                                            meas_sigmas=np.array([10, .5]),
                                            minK=50,
                                            incK=50)

    #=================================================
    # Parameters: 
    # realRobot the mean estimated position of the robot [x,y,theta]
    # realCov the covariance of the robot position [3,3] square matrix
    #=================================================
    for t in range(numSteps):
        u,z = soccer_bot.step(mu,cov)
        # print("odometry\n", u)
        # print("measurement:\n", z)
        # mu,cov = ekf_SLAM_alg.step(u,z)
        # print(mu)
        # print("mean\n", mu)
        # print("covariance:\n", cov)
        mu,cov = graph_alg.step(u,z)




if __name__ == "__main__":
    run()
