import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(linewidth=256)
from helpers import compose, wrap_angle


class DataLoader:
    """
    The DataLoader class is a convenient tool for outputting odometry and measurement data. It reads in from the file
    the odometry and measurement data, then acts as an iterator. Each time __next__ is called (e.g. in a for loop) the
    iterator will look at the next odometry and measurement data and output the one which occurs next in time.
    Odometry gets output as a list of [delta x, delta z, delta theta] all expressed in the world frame, and
    measurements get output as a list of tuples, each tuple containing (AR tag ID, AR tag range, AR tag bearing).
    """
    def __init__(self):
        
        # # Load measurment data
        with open('measurement_data.txt') as f:
            m_txt = f.readlines()
        # remove new line characters
        m_txt = [x.strip() for x in m_txt]

        self.mdata = []
        for m in m_txt:
            k = self.parseText(m)
            if k != None:
                line = self.parseText(m)
                if line is not None:
                    self.mdata.append(line)
        # Data is parsed as a list, each element of which corresponds to a specific time. Each element is as follows:
        # self.mdata[i] = (t, listOfMeasurements)
        # listOfMeasurements is a list, each element of which is like this:
        # listOfMeasurements[0] = (ID, range (m), bearing (rad))
        # So a whole element of the data set, for a single time, would look something like this:
        # self.mdata[17] = (5.67, [(16, 5, 0.2), (17, 20, 0.1)]) corresponding to time = 5.67, one measurment of AR 16 at range 5m bearing 0.2 rad, one measurement of AR 17 at range 20, etc. etc.


        # # Load odometry data (much easier lol)
        self.odata = np.load('position_odometry_data.npy')
        # odometry data is an n x 5 numpy array with [t, x, y, z, theta]

        self.m = len(self.mdata)
        self.n = len(self.odata)

        self.i = 0
        self.j = 0
        
    def __iter__(self):
        self.i = 0
        self.j = 0
        return self
        
    def __next__(self):

        if self.i >= self.n or self.j >= self.m:
            raise StopIteration
        
        else:
            odometry = None
            measurement = None

            t_odometry = self.odata[self.i][0]
            t_measurement = self.mdata[self.j][0]
            if abs(t_odometry - t_measurement) < 1e-5:  # if we have odometry and measurement at the same time
                odometry = self.odata[self.i, 1:4]
                self.i += 1
                measurement = self.mdata[self.j][1]
                self.j += 1
            else:
                if t_odometry < t_measurement:
                    odometry = self.odata[self.i, 1:4]
                    self.i += 1
                else:
                    measurement = self.mdata[self.j][1]
                    self.j += 1

            return odometry, measurement
    
    def parseText(self, line: str):

        t_start = line.find("[") + 1
        t_end = line.find(",")

        t = 'EMPTY'
        measurements = []

        if t_end != -1:
            t = float(line[t_start:t_end])

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

                measurements.append((id, range, bearing))
            output = (t, measurements)
            return output
        else:
            return None


class EKFFilter:
    def __init__(self):
        self.Q = np.diag([0.5, 0.5 * np.pi / 180])
        # basic measurement uncertainty in terms of d, phi, delta_theta, which will be scaled
        # d is distance rolled, probably very accurate in most instances
        # phi is angle of motion relative to robot frame
        # delta theta is the amount of rotation of the robot
        self.alphas = np.array([0.01, 0.1 * np.pi / 180, 5 * np.pi / 180])
        self.mu = np.array([0, 0, 0], dtype=float)
        self.n = 3  # size of state
        self.m = 0  # number of measurements
        self.cov = np.eye(3) * 0.001

        self.path = self.mu
        self.landmark_dictionary = {}
        self.rangeScale = 1

        self.factorLines = []

    def move(self, delta: np.ndarray):
        r_x = self.mu[0]
        r_z = self.mu[1]
        r_th = self.mu[2]

        delta_x = delta[0]
        delta_z = delta[1]
        delta_th = delta[2]

        d = np.sqrt(delta_x**2 + delta_z**2)
        phi = np.arctan2(delta_z, delta_x)

        mu_new = compose(self.mu[0:3], delta)

        self.mu[0:3] = mu_new

        R = np.array([[self.alphas[0] * np.abs(d), 0, 0],
                      [0, self.alphas[1] * np.abs(phi) * np.abs(d), 0],
                      [0, 0, self.alphas[2] * np.abs(delta_th)]])

        # R = np.eye(3)

        A = np.eye(self.n)
        A[0:3, 0:3] = np.array([[1, 0, -d * np.sin(r_th + phi)],
                               [0, 1, d * np.cos(r_th + phi)],
                               [0, 0, 1]])

        # A[0:3, 0:3] = np.eye(3)

        V = np.zeros((self.n, 3))
        V[0:3, 0:3] = np.array([[np.cos(r_th + phi), np.sin(r_th + phi), 0],
                                [-d * np.sin(r_th + phi), d * np.cos(r_th + phi), 0],
                                [0, 0, 1]])

        # V[0:3, 0:3] = np.eye(3)

        self.cov = A @ self.cov @ A.T + V @ R @ V.T
        self.cov = 0.5 * (self.cov + self.cov.T)
        # update the covariance

        self.path = np.vstack([self.path, self.mu[0:3]])
    
    def correct(self, correctionList):
        m = len(correctionList)
        if m == 0:
            return

        H = np.zeros((m * 2, self.n))
        Q = np.zeros((2 * m, 2 * m))
        zhat = np.zeros((m * 2,))
        z = np.zeros((m * 2,))

        IDs = []

        for i, measurement in enumerate(correctionList):
            ID = measurement[0]
            IDs.append(ID)
            j = self.landmark_dictionary[ID]
            r = measurement[1] / self.rangeScale
            b = measurement[2]
            z[2 * i:2 * i + 2] = np.array([r, b])

            mu_landmark = self.mu[3 + 2 * j:3 + 2 * j + 2]
            dx = mu_landmark[0] - self.mu[0]
            dz = mu_landmark[1] - self.mu[1]

            q = dx**2 + dz**2

            zhat[2 * i:2 * i + 2] = np.array([np.sqrt(q),
                                              np.arctan2(dz, dx) + self.mu[2]])
            H[2 * i:2 * i + 2, 0:3] = np.array([[-dx / np.sqrt(q), -dz / np.sqrt(q), 0],
                                                [dz / q, -dx / q, -1]])
            H[2 * i:2 * i + 2, 3 + 2 * j:3 + 2 * j + 2] = np.array([[dx / np.sqrt(q), dz / np.sqrt(q)],
                                                                    [-dz, dx]])

            Q[2 * i:2 * i + 2, 2 * i:2 * i + 2] = self.Q

            l_mu = np.array([self.mu[0] + np.cos(self.mu[2] + b) * r,
                             self.mu[1] + np.sin(self.mu[2] + b) * r])

            self.factorLines.append((ID, np.vstack([self.mu[0:2], l_mu])))

        S = H @ self.cov @ H.T + Q
        K = self.cov @ H.T @ np.linalg.inv(S)

        # self.mu = self.mu + K @ (z - zhat)
        # self.cov = self.cov - K @ H @ self.cov

        print("Updated with measurements: ", IDs)

    def augment(self, augmentList):
        m = len(augmentList)
        if m == 0:
            return

        for measurement in augmentList:
            ID = measurement[0]
            r = measurement[1] / self.rangeScale
            b = measurement[2]

            j = len(self.landmark_dictionary.keys())
            print(f"Adding landmark {ID} as state {j}")
            self.landmark_dictionary[ID] = j

            l_mu = np.array([self.mu[0] + np.cos(self.mu[2] + b) * r,
                             self.mu[1] + np.sin(self.mu[2] + b) * r])

            G_r = np.array([[1, 0, -r * np.sin(self.mu[2] + b)],
                            [0, 1, r * np.cos(self.mu[2] + b)]])

            G_l = np.array([[np.cos(self.mu[2] + b), -r * np.sin(self.mu[2] + b)],
                            [np.sin(self.mu[2] + b), r * np.cos(self.mu[2] + b)]])

            S_rr = self.cov[0:3, 0:3]
            S_ll = G_r @ S_rr @ G_r.T + G_l @ self.Q @ G_l.T
            S_lr = G_r @ S_rr
            S_rl = S_lr.T

            if self.n == 3:  # first landmark
                self.cov = np.block([[S_rr, S_rl],
                                     [S_lr, S_ll]])

            else:
                S_LL = self.cov[3:None, 3:None]
                S_rL = self.cov[0:3, 3:None]
                S_Lr = S_rL.T
                S_lL = G_r @ S_rL
                S_Ll = S_lL.T
                self.cov = np.block([[S_rr, S_rL, S_rl],
                                     [S_Lr, S_LL, S_Ll],
                                     [S_lr, S_lL, S_ll]])

            self.mu = np.hstack([self.mu, l_mu])
            self.n += 2
            self.m += 1

            self.factorLines.append((ID, np.vstack([self.mu[0:2], l_mu])))

    def update(self, u):
        odometry = u[0]
        measurements = u[1]

        if odometry is not None:
            self.move(odometry)

        if measurements is not None:
            augmentList = []
            correctionList = []
            for z in measurements:
                tagID = z[0]
                if tagID in self.landmark_dictionary.keys():
                    correctionList.append(z)
                else:
                    augmentList.append(z)
            self.correct(correctionList)
            self.augment(augmentList)
        

class HWVizualization:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.path, = self.ax.plot([0, 1], [0, 1], c='k', ls='--')
        self.robotCov, = self.ax.plot([0, 1,], [0, 1], c='r')
        self.landmarkCovs = []
        
        theta = np.linspace(0, 2 * np.pi, 20)
        self.circle = np.array([[np.cos(th), np.sin(th)] for th in theta])

        self.robotPoints = np.array([[0.5, 0.5],
                                     [-0.5, 0.5],
                                     [-0.5, -0.5],
                                     [0.5, -0.5],
                                     [0.5, 0.5],
                                     [0.85, 0],
                                     [0.5, -0.5]]) * 1.0

        self.robotPlot, = self.ax.plot(self.robotPoints[:, 0], self.robotPoints[:, 1], c='k')

        plt.pause(0.1)

        self.ax.axis('square')

        plt.pause(0.1)

        self.colors = []
    
    def update(self, slam: EKFFilter):
        self.path.set_data(slam.path[:, 0], slam.path[:, 1])
        covPoints = self.get_ellipse(slam.mu[0:2], slam.cov[0:2, 0:2])
        self.robotCov.set_data(covPoints[:, 0], covPoints[:, 1])
        xmin, xmax = np.min(slam.path[:, 0]), np.max(slam.path[:, 0])
        ymin, ymax = np.min(slam.path[:, 1]), np.max(slam.path[:, 1])

        # for i in range(slam.m):
        #     covPoints = self.get_ellipse(slam.mu[3+2*i:3+2*i+2],
        #                                  slam.cov[3+2*i:3+2*i+2, 3+2*i:3+2*i+2])
        #     if len(self.landmarkCovs) < i + 1:
        #         cov, = self.ax.plot(covPoints[:, 0], covPoints[:, 1], c='m')
        #         self.landmarkCovs.append(cov)
        #         self.ax.plot([slam.mu[0], slam.mu[3+2*i]], [slam.mu[1], slam.mu[3+2*i+1]], c='y')
        #     else:
        #         self.landmarkCovs[i].set_data(covPoints[:, 0], covPoints[:, 1])

        for data in slam.factorLines:
            ID, points = data
            j = slam.landmark_dictionary[ID]
            if j > len(self.colors) - 1:
                cnew = np.random.random((4,))
                cnew[3] = 0.5
                cnew[0:3] = cnew[0:3] / np.max(cnew[0:3])
                self.colors.append(cnew)
                self.ax.plot(points[:, 0], points[:, 1], c=self.colors[j], label=f"ID {ID}")
                plt.legend()
            else:
                self.ax.plot(points[:, 0], points[:, 1], c=self.colors[j])

        slam.factorLines = []

        th = slam.mu[2]
        p = self.robotPoints @ np.array([[np.cos(th), -np.sin(th)],
                                         [np.sin(th), np.cos(th)]]).T + slam.mu[0:2]

        self.robotPlot.set_data(p[:, 0], p[:, 1])

        self.ax.set_xlim([xmin - 10, xmax + 10])
        self.ax.set_ylim([ymin - 10, ymax + 10])
        plt.pause(0.001)
    
    def get_ellipse(self, center, A):
        E, V = np.linalg.eig(A)
        i = 0 if E[0] > E[1] else 1
        j = 1 - i
        angle = np.arctan2(V[:, i][1], V[:, i][0])
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]])
        scale = np.sqrt(E)

        p = np.copy(self.circle)
        p[:, 0] = p[:, 0] * 3 * scale[i]
        p[:, 1] = p[:, 1] * 3 * scale[j]

        p = p @ R.T + center
        return p


if __name__ == '__main__':

    # x = np.load('delta_odometry_data.npy')
    # print(x)

    loader = DataLoader()
    print(f"{loader.odata.shape[0]} odometry inputs, {len(loader.mdata)} measurement inputs")
    slammer = EKFFilter()
    viz = HWVizualization()
    for x in loader:
        slammer.update(x)
        viz.update(slammer)




