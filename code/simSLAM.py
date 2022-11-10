import numpy as np
from generator import generate
import time
from fieldSettings import field
import helpers


np.set_printoptions(linewidth=256)
import matplotlib

# this might be necessary on MacOS for live plotting, dunno what it'll do on other systems
matplotlib.use("Qt5Agg")

import matplotlib.pyplot as plt
dpi = 147
plt.rcParams["figure.figsize"] = (4 * dpi/80, 4 * dpi/80) # size of plots, set to 8x8 inches (default 80ppi, so might be be the same on a given display)

# Can change this to true if you need some debugs in your code, I did it on specific timesteps where I had issues
helpers.printDebug = False 
from helpers import debugPrint

class simulation:
    def __init__(self, numSteps=100, pauseLen=0.0, makeGif=True):
        self.numSteps = numSteps
        self.pauseLen = pauseLen
        self.makeGif = makeGif
        self.t = 0

        # initialize and settings
        self.initialStateMean = np.array([180., 50., 0.]) # initial state, [x, y, theta]
        maxObs = 3 # The maximum number of features that will be observed at any timestep
        alphas = np.array([0.05, 0.001, 0.05, 0.01])**2 # These have to do with the error in our motion controls
        betas = np.array([10, 10*np.pi/180]) # Error in observations
        R = np.diag(betas**2)
        deltaT = 0.1 # Time between each timestep, must be < 1.0s
        
        # NOTE: The below will pull a previously-generated dataset if one exists for the format you've selected.  If this is undesireable, call with "forceNew=True"
        self.data = generate(self.initialStateMean, numSteps, alphas, betas, deltaT, maxObs, forceNew=False) # there are other options in this function def for what landmarks are observed, we won't mess with them


        self.muHist = np.zeros((numSteps, 2)) # Tracking the position of the robot over time for plotting


        plt.ion() # Interacting plotting so we can SEE it doing its thing
        self.gifFrames = []


    def step(self, realRobot=np.array([180., 50., 0.]),realCov= 1e-03*np.eye(3)):
        numSteps = self.numSteps
        makeGif = self.makeGif
        data = self.data
        initialStateMean = self.initialStateMean
        t = self.t
 
        # print("Running step ", t, " currently have ", int((len(realRobot)-3)/2), " landmarks ") # Not necessary, just nice to see.

        #=================================================
        #TODO: plot and evaluate filter results here
        #=================================================
        self.muHist[t] = realRobot[:2] # Track the position of the robot over time

        self.plotsim(data, t, initialStateMean, realRobot, realCov, self.muHist); # Plot the state as it is after the timestep
        plt.legend()        
        plt.gcf().canvas.draw() # Tell the canvas to draw, interactive mode is weird
        if self.pauseLen > 0: # If we've got a pauselen, let's take a break so it doesn't blur past
            time.sleep(self.pauseLen)

        # Save GIF Data
        if (makeGif): # If we're saving to make a video, let's put the current frame into a saved image for later processing.
            # Video/GIF generation
            from PIL import Image # we need PIL to do so
            imgData = np.frombuffer(plt.gcf().canvas.tostring_rgb(), dtype=np.uint8)
            w, h = plt.gcf().canvas.get_width_height()
            mod = np.sqrt(
                imgData.shape[0] / (3 * w * h)
            )  # multi-sampling of pixels on high-res displays does weird things.
            im = imgData.reshape((int(h * mod), int(w * mod), -1))
            self.gifFrames.append(Image.fromarray(im))

        # Make sure the canvas is ready to go for the next step
        plt.gcf().canvas.flush_events() 

        #=================================================
        # Data available to your filter at this time step
        #=================================================
        u = data[t, 0:3] # [d_rot_1, d_trans, d_rot_2]
        temp = data[t, 9:] # [id or -1 if no obs, noisy dist, noisy theta, noise-free dist, noise-free theta, repeat....]
        z = [] # We need to extract measurements.  Simulated data has a "-1" for id if there isn't an observation in that slot for this timestep.
        for i in range(int(len(temp)/5)): # So we need to check each
            if (temp[i*5] != -1): # and see if it's real or not
                z.append(temp[i*5:i*5+3]) # Then grab the first 3 values, since they have noise - last 2 are noise-free, not allowed in our simulator.
        z = np.array(z) # [[id, dist, theta], [id, dist theta], etc]
        order = np.array([1, 2, 0]) # But we want order "dist, theta, id" to match VP.
        z = ((z.T)[order]).T # Reorder, and now our measurements are ready to go.

        # GIF Plotting
        if (makeGif and t == self.numSteps-1):
                    # Save into a GIF file that loops forever
            self.gifFrames[0].save(
                "gifOutput.gif",
                format="GIF",
                append_images=self.gifFrames[1:],
                save_all=True,
                duration=numSteps * 2 * .01,
                loop=1,
            )
        plt.show() # And just show the last image

        self.t += 1
        return u, z

            
            
        # # END of Loop    
        # plt.ioff() # Once we've done all time steps, turn off interactive mode
        # print("Finished execution") # Nice to know when we're done since sometimes the data moves slowly

        
    #==========================================================================
    # Pretty drawing stuff
    #==========================================================================
    def plotsim(self, data, t, initialStateMean, mu, Sigma, muHist):

        noiseFreePathColor = '#00FF00'
        noisyPathColor = '#0000FF'
        estimatedPathColor = '#FFBB00'

        noiseFreeBearingColor = '#00FFFF'
        observedBearingColor = '#FF0000'
            
        #=================================================
        # data *not* available to your filter, i.e., known
        # only by the simulator, useful for making error plots
        #=================================================
        # actual position (i.e., ground truth)
        x, y, theta = data[t, 3:6]
        
        # real observation
        z = data[t, 9:] # [id or -1 if no obs, noisy dist, noisy theta, noise-free dist, noise-free theta, repeat....]
        obs = [] # We need to gather together the observations that are non-null
        for i in range(int(len(z)/5)):
            if (not (z[i*5] == -1)):
                obs.append(z[i*5:i*5+5])
        obs = np.array(obs)

        #################################################
        # Graphics
        #################################################
        plt.clf() # clear the frame.
        helpers.plotField(obs[:,0]) # Plot the field with the observed landmarks highlighted

        # draw actual path and path that would result if there was no noise in
        # executing the motion command
        plt.plot(np.array([initialStateMean[0], *data[:t, 6]]), np.array([initialStateMean[1], *data[:t, 7]]), color=noiseFreePathColor, label='Noise Free Path')
        plt.plot(data[t, 6], data[t, 7], '*', color=noiseFreePathColor)

        # draw the path that has resulted from the movement with noise
        plt.plot(np.array([initialStateMean[0], *data[:t, 3]]), np.array([initialStateMean[1], *data[:t, 4]]), color=noisyPathColor, label='True Noisy Path')
        helpers.plotRobot(data[t, 3:6], "black", "#00FFFF40")

        # draw the path the estimated robot followed
        plt.plot(np.array([initialStateMean[0], *muHist[:t, 0]]), np.array([initialStateMean[1], *muHist[:t, 1]]), color=estimatedPathColor, label='EKF SLAM Est')
        plt.plot([mu[0]], [mu[1]], '*', color=estimatedPathColor)
        helpers.plotCov2D(mu[:2], Sigma[:2, :2], color=estimatedPathColor, nSigma=3)

        for observation in obs:
            # indicate observed angle relative to actual position
            plt.plot(np.array([x, x+np.cos(theta + observation[2])*observation[1]]), np.array([y, y+np.sin(theta + observation[2])*observation[1]]), color=observedBearingColor)

            # indicate ideal noise-free angle relative to actual position
            plt.plot(np.array([x, x+np.cos(theta + observation[4])*observation[3]]), np.array([y, y+np.sin(theta + observation[4])*observation[3]]), color=noiseFreeBearingColor)
        
        for i in range(int((len(mu) - 3)/2)):
            # We'll also plot a covariance ellipse for each landmark
            helpers.plotCov2D(mu[3+i*2:3+i*2+2], Sigma[3+i*2:3+i*2+2, 3+i*2:3+i*2+2], nSigma=3)
