import numpy as np
import gtsam
import matplotlib.pyplot as plt
import time 

class graph_slam_known:
    def __init__(self, initialMean, alphas, betas, robust=True):
        # Parameters
        self.minK = 1  # minimum number of range measurements to process initially
        self.incK = 2  # minimum number of range measurements to process after
        sigmaR = 10        # range standard deviation

        self.rng = np.random.default_rng()

        prior_xy_sigma = 0.3
        prior_theta_sigma = 5
        odometry_xy_sigma = 0.2
        odometry_theta_sigma = 5

        PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([prior_xy_sigma,
                                                                prior_xy_sigma,
                                                                prior_theta_sigma*np.pi/180]))
        self.ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([odometry_xy_sigma,
                                                                    odometry_xy_sigma,
                                                                    odometry_theta_sigma*np.pi/180]))
        NM = gtsam.noiseModel
        self.looseNoise = NM.Isotropic.Sigma(2, 1000)     # loose LM prior
        self.MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()

        # Initialize iSAM
        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(0.1)
        self.isam = gtsam.ISAM2(parameters)

        # Add prior on first pose
        pose0 = gtsam.Pose2(*initialMean)

        self.prev_pose_key = gtsam.symbol('X', int(0))
        self.graph.addPriorPose2(self.prev_pose_key, pose0, PRIOR_NOISE)
        self.initial_estimate.insert(self.prev_pose_key, pose0)

        # set some loop variables
        self.i = 1  # step counter
        self.initialized = False
        self.lastPose = pose0

        self.initializedLandmarks = set()
        plt.ion()


    def step(self, odometry, measurements):
        relativePose = gtsam.Pose2(odometry)
        predictedPose = self.lastPose.compose(relativePose)
        pose_key = gtsam.symbol('X', int(self.i))
        
        # add odometry factor
        # self.graph.add(gtsam.BetweenFactorPose2(self.i - 1, self.i, relativePose, self.odoNoise))
        self.graph.add(gtsam.BetweenFactorPose2(self.prev_pose_key, pose_key, relativePose, self.ODOMETRY_NOISE))

        # predict pose and add as initial estimate
        # self.initial_estimate.insert(self.i, predictedPose)
        self.initial_estimate.insert(pose_key, predictedPose)
        self.lastPose = predictedPose

        # Check if there are range factors to be added
        if (len(measurements) > 0):
            for z in measurements:
                id, dist, bearing = z
                landmark_key = gtsam.symbol('L',int(id))
                self.graph.add(gtsam.BearingRangeFactor2D(pose_key, landmark_key, gtsam.Rot2(bearing), dist, self.MEASUREMENT_NOISE))
                # self.graph.add(gtsam.BearingRangeFactor2D(self.i, landmark_key, gtsam.Rot2(bearing), dist, self.MEASUREMENT_NOISE))

                if landmark_key not in self.initializedLandmarks:
                    p = self.rng.normal(loc=0, scale=100, size=(2,))
                    self.initial_estimate.insert(landmark_key, p)

                    print(f"Adding landmark L{id}")
                    self.initializedLandmarks.add(landmark_key)
                    # # We also add a very loose prior on the landmark in case there is only
                    # # one sighting, which cannot fully determine the landmark.
                    self.graph.add(gtsam.PriorFactorPoint2(landmark_key, gtsam.Point2(0, 0), self.looseNoise))
                
        # Check whether to update iSAM 2
        if not self.initialized:  # Do a full optimize for first minK ranges
            print(f"Initializing at time {self.i}")
            batchOptimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
            self.initial_estimate = batchOptimizer.optimize()
            self.initialized = True

        self.isam.update(self.graph, self.initial_estimate)
        result = self.isam.calculateEstimate()
        self.lastPose = result.atPose2(pose_key)

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.countK = 0

        self.prev_pose_key = pose_key
        self.i += 1

        return result


    def finalize(self):
        finalResult = self.isam.calculateEstimate()

        for j in range(1,38):
            landmark_key = gtsam.symbol('L', j)
            p = finalResult.atPoint2(landmark_key)
            print(f"{landmark_key}: {p}")


    def plot_step(self, result):
        poses = gtsam.utilities.allPose2s(result)
        landmarks = gtsam.utilities.extractPoint2(result)
        positions = np.array([poses.atPose2(key).translation()
                            for key in poses.keys()])
        plt.clf() # clear the frame.
        plt.axis(np.array([-200, 200, -200, 200]))
    
        plt.scatter(x=landmarks[:,0], y=landmarks[:,1], color='red')
        plt.plot(positions[:,0], positions[:,1], color='black')
        plt.gcf().canvas.draw() # interactive plotting is weird, but force it to execute here
        plt.gcf().canvas.flush_events() # Make sure the canvas is ready to go for the next step


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
