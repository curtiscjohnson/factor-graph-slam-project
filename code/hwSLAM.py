import numpy as np
import gtsam
import time
import matplotlib.pyplot as plt
import helpers
from PIL import Image


class graph_slam_known:
    def __init__(self, initialMean, alphas, betas, robust=True):
        # Parameters
        self.minK = 50  # minimum number of range measurements to process initially
        self.incK = 15  # minimum number of range measurements to process after
        sigmaR = 100        # range standard deviation

        self.rng = np.random.default_rng()

        NM = gtsam.noiseModel
        priorNoise = NM.Diagonal.Sigmas(gtsam.Point3(alphas))  # prior
        self.looseNoise = NM.Isotropic.Sigma(2, 1000)     # loose LM prior
        self.odoNoise = NM.Diagonal.Sigmas(gtsam.Point3(betas))     # odometry
        gaussian = NM.Isotropic.Sigma(1, sigmaR)     # non-robust
        tukey = NM.Robust.Create(NM.mEstimator.Tukey.Create(15), gaussian)  # robust
        self.rangeNoise = tukey if robust else gaussian
        self.MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))

        # Initialize iSAM
        self.isam = gtsam.ISAM2()

        # Add prior on first pose
        pose0 = gtsam.Pose2(*initialMean)
        self.newFactors = gtsam.NonlinearFactorGraph()
        self.newFactors.addPriorPose2(0, pose0, priorNoise)
        self.initial_estimate = gtsam.Values()
        self.initial_estimate.insert(0, pose0)

        # set some loop variables
        self.i = 1  # step counter
        self.k = 0  # range measurement counter
        self.initialized = False
        self.lastPose = pose0
        self.countK = 0

        self.initializedLandmarks = set()


    def step(self, odometry, measurements):
        # set some loop variables
        i = self.i # step counter
        k = self.k  # range measurement counter
        countK = self.countK

        # Check if there is odometry data
        if len(odometry) > 0:
            lastPose = self.lastPose
            relativePose = gtsam.Pose2(odometry)

            # add odometry factor
            self.newFactors.add(gtsam.BetweenFactorPose2(i - 1, i, relativePose, self.odoNoise))

            # predict pose and add as initial estimate
            predictedPose = lastPose.compose(relativePose)
            lastPose = predictedPose
            self.initial_estimate.insert(i, predictedPose)

        # Check if there are range factors to be added
        if (len(measurements) > 0):
            for z in measurements:
                id, dist, bearing = z
                landmark_key = gtsam.symbol('L',int(id))
                self.newFactors.add(gtsam.BearingRangeFactor2D(i, landmark_key, gtsam.Rot2(bearing),  dist, self.MEASUREMENT_NOISE))
                if landmark_key not in self.initializedLandmarks:
                    p = self.rng.normal(loc=0, scale=100, size=(2,))
                    self.initial_estimate.insert(landmark_key, p)
                    print(f"Adding landmark L{id}")
                    self.initializedLandmarks.add(landmark_key)
                    # We also add a very loose prior on the landmark in case there is only
                    # one sighting, which cannot fully determine the landmark.
                    self.newFactors.add(gtsam.PriorFactorPoint2(landmark_key, gtsam.Point2(0, 0), self.looseNoise))
                k = k + 1
                self.countK = countK + 1

        # Check whether to update iSAM 2
        if (k > self.minK) and (countK > self.incK):
            if not self.initialized:  # Do a full optimize for first minK ranges
                print(f"Initializing at time {k}")
                batchOptimizer = gtsam.LevenbergMarquardtOptimizer(self.newFactors, initial)
                initial = batchOptimizer.optimize()
                self.initialized = True

            self.isam.update(self.newFactors, initial)
            result = self.isam.calculateEstimate()
            lastPose = result.atPose2(i)
            self.newFactors = gtsam.NonlinearFactorGraph()
            initial = gtsam.Values()
            self.countK = 0
        
        result = self.isam.calculateEstimate()

        self.i = self.i + 1



    def determine_loop_closure(self, odom: np.ndarray, current_estimate: gtsam.Values, key: int, xy_tol=0.6, theta_tol=17) -> int:
        """Simple brute force approach which iterates through previous states
        and checks for loop closure.

        Args:
            odom: Vector representing noisy odometry (x, y, theta) measurement in the body frame.
            current_estimate: The current estimates computed by iSAM2.
            key: Key corresponding to the current state estimate of the robot.
            xy_tol: Optional argument for the x-y measurement tolerance, in meters.
            theta_tol: Optional argument for the theta measurement tolerance, in degrees.
        Returns:
            k: The key of the state which is helping add the loop closure constraint.
                If loop closure is not found, then None is returned.
        """
        if current_estimate:
            prev_est = current_estimate.atPose2(key+1)
            rotated_odom = prev_est.rotation().matrix() @ odom[:2]
            curr_xy = np.array([prev_est.x() + rotated_odom[0],
                                prev_est.y() + rotated_odom[1]])
            curr_theta = prev_est.theta() + odom[2]
            for k in range(1, key+1):
                pose_xy = np.array([current_estimate.atPose2(k).x(),
                                    current_estimate.atPose2(k).y()])
                pose_theta = current_estimate.atPose2(k).theta()
                if (abs(pose_xy - curr_xy) <= xy_tol).all() and \
                    (abs(pose_theta - curr_theta) <= theta_tol*np.pi/180):
                        return k


class hw_plot:
    def __init__(self, pauseLen=0.01, makeGif=False):
        self.robot_path = []
        self.pauseLen = pauseLen
        self.makeGif = makeGif
        plt.ion()
        self.gifFrames = []

    def plot_step(self, mu, cov):
        self.robot_path.append(mu[:2])

        self.graphics(mu, cov, self.robot_path, boundingBox=np.array([-100, 200, -50, 200])) # Handle any graphics (keep minimal for speed, 7249 timesteps in the whole dataset)
        plt.gcf().canvas.draw() # interactive plotting is weird, but force it to execute here
        if self.pauseLen > 0: # If we have a pauselen (don't recommend)
            time.sleep(self.pauseLen) # we'll pause here so the frames don't blur by too fast (NOT LIKELY)
        if (self.makeGif):
            imgData = np.frombuffer(plt.gcf().canvas.tostring_rgb(), dtype=np.uint8)
            w, h = plt.gcf().canvas.get_width_height()
            mod = np.sqrt(imgData.shape[0]/(3*w*h)) # multi-sampling of pixels on high-res displays does weird things.
            im = imgData.reshape((int(h*mod), int(w*mod), -1))
            self.gifFrames.append(Image.fromarray(im))

        plt.gcf().canvas.flush_events() # Make sure the canvas is ready to go for the next step

    def graphics(self, mu, Sigma, robotPath, boundingBox=None):
        plt.clf() # clear the frame.
        # restrict view to a bounding box around the current pose
        if boundingBox is not None:
            boundingBox = np.array([boundingBox])
            boundingBox = boundingBox.reshape(-1)
            if (len(boundingBox) == 1):
                plt.axis([*([-boundingBox,boundingBox]+mu[0]), *([-boundingBox,boundingBox]+mu[1])])
            else:
                plt.axis(boundingBox)
        plt.gca().set_aspect('equal', adjustable='box') # Make sure the aspect ratio in the plot is 1:1
        
        estimatedPathColor = '#FFBB00'
    
        # plot the robot's 3-sigma covariance ellipsoid and path
        plt.plot(*robotPath, color=estimatedPathColor)
        helpers.plotCov2D(mu[:2], Sigma[:2, :2], color=estimatedPathColor, nSigma=3)

        for i in range(int((len(mu) - 3)/2)):
            # Plot a covariance ellipse for every landmark, too
            helpers.plotCov2D(mu[3+i*2:3+i*2+2], Sigma[3+i*2:3+i*2+2, 3+i*2:3+i*2+2], nSigma=3)
            # and an indicator 'cause those ellipses get SMALL (especially w.r.t. the size of the area we're mapping)
            plt.plot(*mu[3+i*2:3+i*2+2], "*", color="green")

        # plot the robot (so it's on top))
        helpers.plotRobot(mu[:3], "black", estimatedPathColor, r=0.5) # this is, uh... not what the vehicle really looks like.  But I'm lazy.


def parseText(line: str):
    output = []

    t_start = line.find("[") + 1
    t_end = line.find(",")
    if t_end != -1:
        t = float(line[t_start:t_end])
        output.append(t)
        test_line = line[t_end+1:]

        # Split into measurements
        new_line = list(filter(None, test_line.split("]")))
        
        for l in new_line:
            l = l[1:] if l[0] == ',' else l
            id_start = l.find("[") + 1
            id_end = l.find(",")
            r_end = l.rfind(",")

            id = int(l[id_start:id_end])
            range = float(l[id_end+2:r_end])
            bearing = float(l[r_end+2:])

            output.append([id, range, bearing])
        return output

def get_measurements():
    with open("measurement_data.txt") as f:
        m_txt = f.readlines()
        # remove new line characters
        m_txt = [x.strip() for x in m_txt]

    measurements = []
    for m in m_txt:
        k = parseText(m)
        if k != None:
            measurements.append(parseText(m))
    
    return measurements
