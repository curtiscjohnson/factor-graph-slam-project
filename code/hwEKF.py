import numpy as np


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



if __name__ == '__main__':
    loader = DataLoader()

    for x in loader:
        print(x)


