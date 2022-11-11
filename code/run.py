#!/usr/bin/env python3
import numpy as np
import argparse
from simSLAM import simulation
from simSLAM import ekf_slam

def run():

    numSteps = 50
    # System Noise parameters
    alphas = np.array([0.05, 0.001, 0.05, 0.01])**2 # These have to do with the error in our motion controls
    betas = np.array([10, 10*np.pi/180]) # Error in observations
    soccer_bot = simulation(numSteps=numSteps,alphas=alphas,betas=betas)

    mu = soccer_bot.get_initialStateMean()
    print(mu)
    cov= 1e-03*np.eye(3) # start covariance low but non-zero
    SLAM_alg = ekf_slam(mu,cov,alphas=alphas,betas=betas)

    #=================================================
    # Parameters: 
    # realRobot the mean estimated position of the robot [x,y,theta]
    # realCov the covariance of the robot position [3,3] square matrix
    #=================================================
    for t in range(numSteps):
        u,z = soccer_bot.step(mu,cov)
        # print("odometry\n", u)
        # print("measurement:\n", z)
        mu,cov = SLAM_alg.step(u,z)
        # print("mean\n", mu)
        print("covariance:\n", cov)




if __name__ == "__main__":
    run()
