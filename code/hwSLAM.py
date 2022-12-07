import numpy as np
import gtsam
import matplotlib.pyplot as plt
import time 
import plotly.express as px


class graph_slam_known:
    def __init__(self, initialMean, 
                prior_sigmas=np.array([0, 0, 0]), 
                odo_sigmas=np.array([10, 10, 0.2]), 
                loose_sigma=10, 
                meas_sigmas=np.array([100, (10*np.pi/180)**2]),
                minK=50,
                incK=10):

        # Set up Noise Parameters
        NM = gtsam.noiseModel
        PRIOR_NOISE = NM.Diagonal.Sigmas(prior_sigmas)
        self.ODOMETRY_NOISE = NM.Diagonal.Sigmas(odo_sigmas)
        self.looseNoise = NM.Isotropic.Sigma(2, loose_sigma)
        self.MEASUREMENT_NOISE = NM.Diagonal.Sigmas(meas_sigmas)


        # Initialize iSAM
        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(0.1)
        self.isam = gtsam.ISAM2(parameters)

        self.graph = gtsam.NonlinearFactorGraph()
        self.last_graph = self.graph
        self.initial_estimate = gtsam.Values()


        # Add prior on first pose
        pose0 = gtsam.Pose2(*initialMean)
        self.graph.addPriorPose2(0, pose0, PRIOR_NOISE)
        self.initial_estimate.insert(0, pose0)


        # Parameters
        self.minK = minK  # minimum number of range measurements to process initially
        self.incK = incK  # minimum number of range measurements to process after

        # set some loop variables
        self.i = 1  # step counter
        self.countK = 0
        self.k = 0
        self.initialized = False
        self.lastOdom = np.array([0, 0, 0])
        self.initializedLandmarks = set()

        #plotting stuff
        self.robot_poses = []
        self.landmarks = []
        plt.ion()


    def step(self, odometry, measurements):
        relativePose = gtsam.Pose2(odometry-self.lastOdom)
        self.lastOdom = odometry

        # add odometry factor
        self.graph.add(gtsam.BetweenFactorPose2(self.i-1, self.i, relativePose, self.ODOMETRY_NOISE))

        # predict pose and add as initial estimate
        # predictedPose = self.lastPose.compose(relativePose)
        predictedPose = gtsam.Pose2(*odometry)
        self.initial_estimate.insert(self.i, predictedPose)

        # Check if there are range factors to be added
        if (len(measurements) > 0):
            for z in measurements:
                id, dist, bearing = z
                landmark_key = gtsam.symbol('L',int(id))
                self.graph.add(gtsam.BearingRangeFactor2D(self.i, landmark_key, gtsam.Rot2(bearing), dist, self.MEASUREMENT_NOISE))

                if landmark_key not in self.initializedLandmarks:
                    estimated_L_xy = [predictedPose.x()+dist*np.cos(bearing+predictedPose.theta()),
                                      predictedPose.y()+dist*np.sin(bearing+predictedPose.theta())]
                    self.initial_estimate.insert(landmark_key, estimated_L_xy)

                    # print(f"Adding landmark L{id} at {estimated_L_xy}")
                    self.initializedLandmarks.add(landmark_key)
                    # # We also add a very loose prior on the landmark in case there is only
                    # # one sighting, which cannot fully determine the landmark.
                    self.graph.add(gtsam.PriorFactorPoint2(landmark_key, estimated_L_xy, self.looseNoise))
                self.k += 1
                self.countK += 1


        # Check whether to update iSAM 2
        # if (self.k > self.minK) and (self.countK > self.incK):
        if not self.initialized:  # Do a full optimize for first minK ranges
            # print(f"Initializing at time {self.k}")
            params = gtsam.LevenbergMarquardtParams()
            batchOptimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate, params)
            self.initial_estimate = batchOptimizer.optimize()
            self.initialized = True

        self.isam.update(self.graph, self.initial_estimate)
        current_estimate = self.isam.calculateEstimate()
        lastPose = current_estimate.atPose2(self.i)

        # marginals = gtsam.Marginals(self.factor_graph, current_estimate)
        # self.realCov = marginals.marginalCovariance(i)

        self.last_graph = self.graph
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.countK = 0

        # print(np.array([lastPose.x(), lastPose.y(), lastPose.theta()]))

        self.i += 1
        return current_estimate


    def plot_step(self, current_estimate):
        poses = gtsam.utilities.allPose2s(current_estimate)
        landmarks = gtsam.utilities.extractPoint2(current_estimate)
        positions = np.array([poses.atPose2(key).translation()
                            for key in poses.keys()])


        lastPose = current_estimate.atPose2(self.i-1)
        self.robot_poses.append([lastPose.x(), lastPose.y(), lastPose.theta()])
            

        plt.clf() # clear the frame.
        plt.axis(np.array([-100, 100, -100, 100]))
    
        plt.scatter(landmarks[:,0], landmarks[:,1], 5, color='red')
        plt.plot(positions[:,0], positions[:,1], color='black')

        # plt.plot([pose[0] for pose in self.robot_poses], [pose[1] for pose in self.robot_poses], color='black')
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
