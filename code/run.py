#!/usr/bin/env python3
import numpy as np
import argparse
from simSLAM import simulation




def run():
    numSteps = 10

    soccer_bot = simulation(numSteps=numSteps)
    #=================================================
    # Parameters: 
    # realRobot the mean estimated position of the robot [x,y,theta]
    # realCov the covariance of the robot position [3,3] square matrix
    #=================================================
    for t in range(numSteps):
        u,z = soccer_bot.step()
        print("odometry\n", u)
        print("measurement:\n", z)


if __name__ == "__main__":
    run()
