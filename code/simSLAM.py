import numpy as np
from generator import generate
import time
from fieldSettings import field
import helpers

import math
import gtsam
from gtsam import Point2, Pose2
import gtsam.utils.plot as gtsam_plot
from numpy.random import default_rng
import plotly.express as px
from copy import deepcopy



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
    def __init__(self, numSteps=100, pauseLen=0.0, makeGif=True, alphas=np.array([0.05, 0.001, 0.05, 0.01])**2, betas= np.array([10, 10*np.pi/180])):
        self.numSteps = numSteps
        self.pauseLen = pauseLen
        self.makeGif = makeGif
        self.t = 0

        # initialize and settings
        self.initial_estimateStateMean = np.array([180., 50., 0.]) # initial state, [x, y, theta]
        maxObs = 2 # The maximum number of features that will be observed at any timestep
        # alphas = np.array([0.05, 0.001, 0.05, 0.01])**2 # These have to do with the error in our motion controls
        # betas = np.array([10, 10*np.pi/180]) # Error in observations
        deltaT = 0.1 # Time between each timestep, must be < 1.0s
        
        # NOTE: The below will pull a previously-generated dataset if one exists for the format you've selected.  If this is undesireable, call with "forceNew=True"
        self.data = generate(self.initial_estimateStateMean, numSteps, alphas, betas, deltaT, maxObs, forceNew=True) # there are other options in this function def for what landmarks are observed, we won't mess with them


        self.muHist = np.zeros((numSteps, 2)) # Tracking the position of the robot over time for plotting


        plt.ion() # Interacting plotting so we can SEE it doing its thing
        self.gifFrames = []

    def get_initialStateMean(self):
        return self.initial_estimateStateMean


    def step(self, realRobot=np.array([180., 50., 0.]),realCov= 1e-03*np.eye(3)):
        numSteps = self.numSteps
        makeGif = self.makeGif
        data = self.data
        initialStateMean = self.initial_estimateStateMean
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

        # draw actual path and path that would current_estimate if there was no noise in
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

class ekf_slam:
    def __init__(self,initialSateMean,realCov,alphas,betas, updateMethod='batch'):
        self.R = np.diag(betas**2)
        self.alphas = alphas
        self.realRobot = initialSateMean
        self.realCov = realCov
        self.landmarkSignatures = []
        self.updateMethod = updateMethod


    def step(self, u, z):    
        drot1, dtrans, drot2 = u
        alphas = self.alphas
        R = self.R
        # Setup to Handle Noisy Controls
        M = np.array([[alphas[0]*drot1**2+alphas[1]*dtrans**2,0                                                   ,0                                       ],
                      [0                                     ,alphas[2]*dtrans**2 + alphas[3]*(drot1**2+drot2**2) ,0                                       ],
                      [0                                     ,0                                                   ,alphas[0]*drot2**2 + alphas[1]*dtrans**2]])
        # Implement the predict function defined below
        realRobot_bar, realCov_bar = self.predict(self.realRobot, self.realCov, u, M)
        if (len(z) > 0): # If we HAVE any measurements...
            # Call the associateData function to determine which measurements correspond to which landmarks
            association, H, innovation = self.associateData(z, self.R, realRobot_bar, realCov_bar, self.landmarkSignatures) 
            # Update your state for landmarks already observed
            self.realRobot, self.realCov = self.update(realRobot_bar, realCov_bar, association, H, R, innovation, self.updateMethod) 
            # Augment our state with new landmarks that were not associated
            self.realRobot, self.realCov, self.landmarkSignatures = self.augmentState(association, z, self.realRobot, self.realCov, R, self.landmarkSignatures)
        return self.realRobot, self.realCov
            

    ##################################################
    # Implement the Motion Prediction Equations
    ##################################################
    def motion_model(self,u, mu):
        x,y,theta = mu.flatten()
        dr1, dt, dr2 = u.flatten()

        # predicting motion
        theta = theta + dr1
        x = x + dt*np.cos(theta)
        y = y + dt*np.sin(theta)

        G = np.array([[1,0,-dt*np.sin(theta)],
                    [0,1, dt*np.cos(theta)],
                    [0,0,                1]])
        R = np.array([[-dt*np.sin(theta), np.cos(theta), 0],
                    [ dt*np.cos(theta), np.sin(theta), 0],
                    [1                      ,0       , 1]])

        theta_end = theta + dr2
        # theta_end = helpers.minimizedAngle(theta_end) 
        if theta_end == 1:
            theta_end == 0
            
        return np.array([x, y, theta_end]), G, R


    def predict(self, mu, Sigma, u, M):
        N = int((len(mu)-3)/2)
        # Propogate Jacobian 

        # Find Jacobians

        # Update Mean and Covariance (Handle both Robot State and Landmarks)
        # F = np.hstack([np.eye(3),np.zeros(3,num_landmarks)])
        mu[:3], G, R = self.motion_model(u,mu[:3]) 
        mu_bar = mu
        # G = helpers.block_diag(G,np.eye(num_landmarks*2)) # Inefficient
        F = np.hstack([np.eye(3),np.zeros([3,N*2])])
        G_high = helpers.block_diag(G,np.eye(N*2)) # Inefficient
        Sigma_bar = G_high@Sigma@G_high.T  + F.T@(R @ M @ R.T)@F
            
        
        return mu_bar, Sigma_bar # And give them back to the calling function

    ###############################################
    # TODO: Implement Measurement Update Equations
    ###############################################
    def update(self, mu_bar, Sigma_bar, association, H, Q, innovation, updateMethod):
        ind = np.flatnonzero(np.asarray(association > -1)) # -1 is used as a keyword for "new landmark," -2 is used for "ignore"
        N = mu_bar
        if (len(ind) == 0): # If we don't have any, short-circuit return
            return mu_bar, Sigma_bar

        if (updateMethod == "seq"): #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! untested step through needed
            for i in ind: 
                # TODO: Finish This to update incrementally for each measurement
                # Kalman gain
                S = H[i]@Sigma_bar@H[i].T + Q
                K = (Sigma_bar@H[i].T@ np.linalg.inv(S))
                # Correction
                mu_bar = mu_bar + K@(innovation[i])
                Sigma_bar = (np.eye(len(mu_bar))-K@H[i])@Sigma_bar
                pass # To enable running until implemented
        elif (updateMethod == "batch"): 
            # TODO: Finish This to update all measurements of a time step at once
            H = np.vstack([H[i] for i in ind]) 
            innovation = np.vstack([innovation[i] for i in ind])
            Q_stack = helpers.block_diag(*[Q]*len(ind) )
            # Kalman gain
            S = H@Sigma_bar@H.T + Q_stack 
            K = (Sigma_bar@H.T@ np.linalg.inv(S))
            # Correction
            mu_bar = mu_bar + K@(innovation.flatten())
            Sigma_bar = (np.eye(len(mu_bar))-K@H)@Sigma_bar
            pass # To enable running until implemented        
        else:
            raise Exception("Unknown update method, '" + updateMethod + "' - it must be 'seq' or 'batch'")
        
        return mu_bar, Sigma_bar

    ##################################
    # Implement Augment State
    ##################################
    def augmentState(self, association, measurements, mu, Sigma, Q, landmarkSignatures):
        indices = np.flatnonzero(np.asarray(association == -1)) # If our data association returned a "-1" for a measurement, it's a landmark to add to the state
        measurements = measurements[indices] # We want to filter out only the measurements we cared about
        x, y, theta = mu[:3] # We'll need the robot state
        Sigma_rr = Sigma[:3, :3] # And the main robot covariance, we won't change
        
        # For each measurement of a new landmark update your state
        for z in measurements:
            # Extract info for the measurement
            dist, bearing, sig = z 
            # Update the signatures so we know what landmark index goes to what signature.  Only used for da_known g(mu, z) gives landmark pose
            landmarkSignatures = np.array([*landmarkSignatures, sig])  # use for da_known
            # observed location of a landmark
            landmark_xy =  [x+dist*np.cos(bearing+theta),y+dist*np.sin(bearing+theta)]

            # G = np.asarray([[1,0,dist*np.cos(bearing+theta), np.cos(bearing+theta), dist*np.cos(bearing+theta)],
            #                 [0,1,dist*np.cos(bearing+theta), np.cos(bearing+theta), dist*np.cos(bearing+theta)]]) 
            Gz = np.asarray([[ np.cos(bearing+theta), -dist*np.sin(bearing+theta)],
                            [ np.sin(bearing+theta), dist*np.cos(bearing+theta)]]) 
            Gr = np.asarray([[1, 0,-dist*np.sin(bearing+theta)],
                            [0, 1,dist*np.cos(bearing+theta)]])
            dim = len(Sigma)
            Sigma_LL = Sigma[3:dim, 3:dim]
            Sigma_rL = Sigma[:3, 3:dim]
            Sigma_Lr = Sigma_rL.T
            mu = np.hstack([mu,landmark_xy])
            # Update both the mean (mu_l) and covariance (Sigma_lr, Sigma_rl, Sigma_Ll, Sigma_lL, Sigma_ll) for the new landmark.

            if Sigma_rL.size > 0:
                Sigma_lr = Gr @ Sigma_rr
                Sigma_rl = Sigma_lr.T
                Sigma_lL = Gr @ Sigma_rL
                Sigma_Ll = Sigma_lL.T
                Sigma_ll = Gr @ Sigma_rr @ Gr.T + Gz@Q@Gz.T # comes from measurement noise Q jacobian of g(mu,z) with respect to mu and z (2,3)
                Sigma = np.block([[Sigma_rr, Sigma_rL, Sigma_rl],
                                [Sigma_Lr, Sigma_LL, Sigma_Ll],
                                [Sigma_lr, Sigma_lL, Sigma_ll]])
            else:
                Sigma_lr = Gr @ Sigma_rr
                Sigma_rl = Sigma_lr.T
                Sigma_ll = Gr @ Sigma_rr @ Gr.T + Gz@Q@Gz.T 
                Sigma = np.block([[Sigma_rr, Sigma_rl],
                                [Sigma_lr, Sigma_ll]])
            
        
        return mu, Sigma, landmarkSignatures

    ####################################
    # Implement Data Association
    ####################################
    # Output:
    #  - an "association" vector that specifies the landmark index of each measurement. (-1 means new landmark, -2 means ignore)
    #  - Since MOST data associations (NN, JCBB) end up computing innovation and measurement jacobians,
    #    it can save computation to pass them on from this function. 
    #
    # Inputs:
    #  - measurements obtained at this time step
    #  - R covariance matrix
    #  - Current state estimate (mu, Sigma)Sigma_Lr
    #  - Landmark signatures (only available for use in da_known)
    #  - Short circuit threshhold may be helpful for use in jcbb, not needed in the other algorithms
        # Short circuit threshhold is used in jcbb, not implemented here
    def associateData(self, measurements, R, mu, Sigma, landmarkSignatures = [], shortCircuitThresh = 40.0**2):
        association = [] # Each index of this will hold the index of the landmark in the state.
        innovation = [] # Indexed the same as the above, this is used in the update step
        H = [] # Indexed the same as the above, this is used in the update step
        
        x, y, theta = mu[:3] # We'll use this for innovation and H, even if not for data association.

        for z in measurements: # OK, for each measurement we have...
            dist, bearing, sig = z # Let's extract the components
            if sig in landmarkSignatures: # If we've seen this signature (markerID) before, it's in the state

                # Association
                ind = np.flatnonzero(np.asarray(landmarkSignatures == sig))[0] # We just need to find WHERE in the state
                association.append(ind) # And make sure we return that

                # Innovation
                # TODO: Calculate the Innovation (difference between expected and actual measurement) HERE
                marker_x = mu[ind*2+3]
                marker_y = mu[ind*2+4]
                mu_bar_x, mu_bar_y, mu_bar_t = mu[:3].flatten()
                dx = marker_x-mu_bar_x
                dy = marker_y-mu_bar_y
                q = (dx)**2 + (dy)**2
                z_hat = np.asarray([[np.sqrt(q)],[helpers.minimizedAngle(np.arctan2(dy, dx)-mu_bar_t)]]).reshape(2,)
                innovation.append(z[:2]-z_hat)

                # Calculate the Jacobian HERE
                F = np.zeros([5,len(mu)])
                F[:3,:3] = np.eye(3)
                F[3:,ind*2+3:ind*2+5] = np.eye(2)

                H.append((1/q)*np.asarray([[-1*np.sqrt(q)*dx, -1*np.sqrt(q)*dy,  0, np.sqrt(q)*dx, np.sqrt(q)*dy],
                                        [dy              , -dx             , -q, -dy          , dx           ]])@F)



            # TODO: Read this so you understand what its doing
            else: # If the landmark signature (markerID) HASN'T been seen yet, we need to create a new landmark
                association.append(-1) # We say the association is "-1" to tell future code to add new landmarks
                innovation.append([0, 0]) # We won't use the innovation anyway, but we need to return SOMETHING
                H.append(np.zeros((2, len(mu)))) # Same for our H jacobian

        return np.array(association), np.array(H), np.array(innovation)

class graph_slam_known:
    def __init__(self, initialMean, 
                minK,
                incK,
                alphas,
                betas,
                loose_sigma=5,
                prior_sigmas=np.array([0, 0, 0])):

        # Set up Noise Parameters
        NM = gtsam.noiseModel
        PRIOR_NOISE = NM.Diagonal.Sigmas(prior_sigmas)





        self.looseNoise = NM.Isotropic.Sigma(2, loose_sigma)
        self.MEASUREMENT_NOISE = NM.Diagonal.Sigmas([betas[0]**2+1,betas[1]+.001])
        self.cov = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.robot_cov = np.zeros([3,3])
        self.alphas = alphas

        # Initialize iSAM
        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(0.1)
        self.isam = gtsam.ISAM2(parameters)

        self.factor_graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()

        # Add prior on first pose
        pose0 = gtsam.Pose2(*initialMean)
        print(pose0)
        self.factor_graph.addPriorPose2(0, pose0, PRIOR_NOISE)
        self.initial_estimate.insert(0, pose0)

        self.overall_estimate = deepcopy(self.initial_estimate)
        self.total_graph = deepcopy(self.factor_graph)

        # Parameters
        self.minK = minK  # minimum number of range measurements to process initially
        self.incK = incK  # minimum number of range measurements to process after

        # set some loop variables
        self.i = 1  # step counter
        self.countK = 0
        self.k = 0
        self.initialized = False
        self.prev_pose = initialMean
        self.initial_estimatedLandmarks = set()       

        self.path = np.array([initialMean[0], initialMean[1]]) 


    def step(self, odometry, measurements):
        curr_pose, odometryNoise = self.motion_model(odometry, self.prev_pose)
        sqrMag = np.abs(odometry)**2
        noisyMotion = np.zeros((3,1))
        alphas = self.alphas
        noisyMotion[0] = 10*(alphas[0]*sqrMag[0] + alphas[1]*sqrMag[1])+1#np.random.normal(odometry[0], np.sqrt(alphas[0]*sqrMag[0] + alphas[1]*sqrMag[1]))
        noisyMotion[1] = 10*(alphas[2]*sqrMag[1] + alphas[3]*(sqrMag[0]+sqrMag[2]))+1#np.random.normal(odometry[1], np.sqrt(alphas[2]*sqrMag[1] + alphas[3]*(sqrMag[0]+sqrMag[2])))
        noisyMotion[2] = 10*(alphas[0]*sqrMag[2] + alphas[1]*sqrMag[1])+1#np.random.normal(odometry[2], np.sqrt(alphas[0]*sqrMag[2] + alphas[1]*sqrMag[1]))
        odometryNoise = gtsam.noiseModel.Diagonal.Sigmas(noisyMotion)
        # odometryNoise = gtsam.noiseModel.Gaussian.Covariance(odometryNoise)

        

        # odometryNoise = gtsam.noiseModel.Gaussian.Covariance(helpers.block_diag(noisyMotion))
        relativePose = gtsam.Pose2(curr_pose-self.prev_pose)
        self.prev_pose = curr_pose

        # add odometry factor
        self.factor_graph.add(gtsam.BetweenFactorPose2(self.i-1, self.i, relativePose, odometryNoise))
        self.total_graph.add(gtsam.BetweenFactorPose2(self.i-1, self.i, relativePose, odometryNoise))

        predictedPose = Pose2(curr_pose[0], curr_pose[1],curr_pose[2])
        # self.realRobot = predictedPose
        self.initial_estimate.insert(self.i,predictedPose)
        self.overall_estimate.insert(self.i,predictedPose)

        # Check if there are range factors to be added
        if (len(measurements) > 0):
            for z in measurements:
                dist, bearing, sig = z
                j = sig
                landmark_key = gtsam.symbol("L",int(j))
                self.factor_graph.add(gtsam.BearingRangeFactor2D(self.i, landmark_key, gtsam.Rot2(bearing),  dist, self.MEASUREMENT_NOISE))
                self.total_graph.add(gtsam.BearingRangeFactor2D(self.i, landmark_key, gtsam.Rot2(bearing),  dist, self.MEASUREMENT_NOISE))
                
                if landmark_key not in self.initial_estimatedLandmarks:
                    # minimized_angle = helpers.minimizedAngle(bearing+predictedPose.theta())
                    estimated_L_xy=[predictedPose.x()+dist*np.cos(bearing+predictedPose.theta()),predictedPose.y()+dist*np.sin(bearing+predictedPose.theta())]
                    self.initial_estimate.insert(landmark_key, estimated_L_xy)
                    self.overall_estimate.insert(landmark_key, estimated_L_xy)
                    print(f"Adding landmark L{j} at : {estimated_L_xy}")
                    self.initial_estimatedLandmarks.add(landmark_key)

                    # We also add a very loose prior on the landmark in case there is only
                    # one sighting, which cannot fully determine the landmark.
                    self.factor_graph.add(gtsam.PriorFactorPoint2(landmark_key, estimated_L_xy, self.looseNoise))
                    self.total_graph.add(gtsam.PriorFactorPoint2(landmark_key, estimated_L_xy, self.looseNoise))

                self.k += 1
                self.countK += 1
        cov = self.cov  
        landmark_cov = []  
        mu = curr_pose
        # Check whether to update iSAM 2
        if (self.k > self.minK) and (self.countK > self.incK):
            if not self.initialized:  # Do a full optimize for first minK ranges
                print(f"Initializing at time {self.k}")
                params = gtsam.LevenbergMarquardtParams()
                batchOptimizer = gtsam.LevenbergMarquardtOptimizer(self.total_graph, self.overall_estimate, params)
                self.initial_estimate = batchOptimizer.optimize()
                self.overall_estimate = deepcopy(self.initial_estimate)
                self.initialized = True
                self.countK = 0

            self.isam.update(self.factor_graph, self.initial_estimate)
            current_estimate = self.isam.calculateEstimate()
            lastPose = current_estimate.atPose2(self.i)
            mu = np.array([lastPose.x(), lastPose.y(), lastPose.theta()])
            
            # Get the covariance of the robot
            marginals = gtsam.Marginals(self.total_graph, self.overall_estimate)
            self.robot_cov = marginals.marginalCovariance(self.i)
            for key in self.initial_estimatedLandmarks:
                landmark_cov.append(marginals.marginalCovariance(key))
                mu = np.append(mu,self.overall_estimate.atPoint2(key))


            self.factor_graph = gtsam.NonlinearFactorGraph()
            self.initial_estimate = gtsam.Values()
            

            for c in landmark_cov:
                cov = helpers.block_diag(cov,c)
            cov[:3,:3] = self.robot_cov

            poses = gtsam.utilities.allPose2s(current_estimate)
            self.path = np.array([poses.atPose2(key).translation() for key in poses.keys()])
        else:
            self.path = np.vstack([self.path, mu[0:2]])

        self.i += 1
        return mu, cov, self.path

    ##################################################
    # Implement the Motion Prediction Equations
    ##################################################
    def motion_model(self, u, mu):
        x,y,theta = mu.flatten()
        dr1, dt, dr2 = u.flatten()

        alphas = self.alphas
        # predicting motion
        theta = theta + dr1
        x = x + dt*np.cos(theta)
        y = y + dt*np.sin(theta)

        theta_end = theta + dr2
        # theta_end = helpers.minimizedAngle(theta_end) 

        M = np.array([[alphas[0]*dr1**2+alphas[1]*dt**2,0                                           ,0                                 ],
                      [0                               ,alphas[2]*dt**2 + alphas[3]*(dr1**2+dr2**2) ,0                                 ],
                      [0                               ,0                                           ,alphas[0]*dr2**2 + alphas[1]*dt**2]])
        
        R = np.array([[-dt*np.sin(theta), np.cos(theta), 0],
                      [ dt*np.cos(theta), np.sin(theta), 0],
                      [1                ,0             , 1]])
        G = np.array([[1,0,-dt*np.sin(theta)],
                      [0,1, dt*np.cos(theta)],
                      [0,0,                1]])

        Sigma_odometry =  R @ M @ R.T #+G @ self.robot_cov @ G.T 
            
        return np.array([x, y, theta_end]), Sigma_odometry