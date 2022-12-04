import numpy as np
import matplotlib.pyplot as plt


class DataLoader:
    def __init__(self):
        
        # # Load measurment data
        with open('./code/measurement_data.txt') as f:
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
        self.odata = np.load('./code/position_odometry_data.npy')
        # odometry data is a n x 5 numpy array with [t, x, y, z, theta]

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
                odometry = self.odata[self.i][1:None]
                self.i += 1
                measurement = self.mdata[self.j][1]
                self.j += 1
            else:
                if t_odometry < t_measurement:
                    odometry = self.odata[self.i][1:None]
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
        self.Q = np.diag([1, 0.1])
        self.R = np.diag([1, 1, 0.1])
        self.mu = np.array([0, 0, 0])
        self.cov = np.eye(3) * 0.1

        self.landmark_dictionary = {}

    def move(self, r: np.ndarray):
        delta = r - self.mu[0:3]
        self.mu[0:3] = r
        self.cov = self.cov + self.R
        # update the covariance
    
    def correct(self, z):
        m = len(z)
        if m == 0:
            return

    def augment(self, z):
        m = len(z)
        if m == 0:
            return

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
        self.path, = self.ax.plot(c='k', ls='--')
        self.robotCov, = self.ax.plot(c='r', ls='--')
        self.landmarkCovs = []
        
        theta = np.linspace(0, 2 * np.pi, 20)
        self.circle = np.array([[np.cos(th), np.sin(th)] for th in theta])
    
    def update(self, path, cov):
        self.path.set_data(path[:, 0], path[:, 1])
        covPoints = self.get_ellipse(path[-1], cov)
        self.robotCov.set_data(covPoints[:, 0], covPoints[:, 1])
    
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
        p[:, 1] = p[:, 1] * 3 * scale[i]

        p = p @ R.T
        return p

if __name__ == '__main__':
    loader = DataLoader()

    for x in loader:
        print(x)


