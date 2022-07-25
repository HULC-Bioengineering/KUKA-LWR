'''
Assume OPTOTRAK markers are:
  0: base
  1: tool (wrt base)
  2: stylus (wrt base)
'''
import numpy as np
from numpy.linalg import norm
np.set_printoptions(suppress=True)

POINTS_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\'
CERTUS_RECORDED_POINT_FILES = ['base0.csv', 'base1.csv', 'base2.csv']

def createCoordinateSystemFromCertusLog(filepath, recorded_point_files):
# parse point csv recroded files for average px, py, pz
  points = []  # list of tuples because tuples are immutable
  for point_file in recorded_point_files:
      certus = np.genfromtxt(filepath + point_file, delimiter=',',
                             dtype=np.double, autostrip=True,
                             skip_header=5, usecols=(36,37,38))
      points.append(np.array([np.average(certus[:,0]),
                              np.average(certus[:,1]),
                              np.average(certus[:,2])]))

  # create vectors 1 and 2
  v1 = points[2] - points[1]
  v2 = points[0] - points[1]
  v2 *= -1
  v1 /= (norm(points[2] - points[1]))
  v2 /= (norm(points[0] - points[1]))

  # cross product of 1 and 2 creates v3
  v3 = np.cross(v1,v2)
  v2 = np.cross(v3, v1)

  r = np.hstack((v1.reshape((3,1)),
                 v2.reshape((3,1)),
                 v3.reshape((3,1)),
                 points[1].reshape((3,1))))
  return r
  
if __name__ == "__main__":
    r = createCoordinateSystemFromCertusLog(POINTS_FILEPATH,
                                            CERTUS_RECORDED_POINT_FILES)
    print('(%f,%f,%f,%f), (%f,%f,%f,%f), (%f,%f,%f,%f), (0,0,0,1)' % (r.item(0), r.item(1), r.item(2), r.item(3), r.item(4), r.item(5), r.item(6), r.item(7), r.item(8), r.item(9), r.item(10), r.item(11)))