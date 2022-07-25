import math
import matplotlib as mpl
mpl.use("pgf")
pgf_with_pdflatex = {
    "pgf.texsystem": "pdflatex",
    "pgf.preamble": [
         r"\usepackage[utf8x]{inputenc}",
         r"\usepackage[T1]{fontenc}",
         r"\usepackage{cmbright}",
         ]
}
mpl.rcParams.update(pgf_with_pdflatex)
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import norm

BASE_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\'
GRID = (4,3)
PLOT_SIZE = (6,7)

POINTS_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\'
    
#certus_file = BASE_FILEPATH + 'experiments\\multi_final_v2_1_optical.csv'
certus_file = BASE_FILEPATH + 'experiments\\.csv'

certus = np.genfromtxt(certus_file, delimiter=',', dtype=np.double, autostrip=True,
                       skip_header=5)
                
#certus[:,0] *= 0.002


obj_objc_t_matrix = np.matrix(((-0.044010,-0.996970,-0.064003,-62.576464), (-0.451635,0.076999,-0.888866,0.261632), (0.891117,-0.010213,-0.453655,16.894884), (0,0,0,1)))
obj_objc_t_matrix = inv(obj_objc_t_matrix)
px = []
py = []
pz = []

for x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22 in zip(certus[:,23], certus[:,24], certus[:,25], certus[:,14], certus[:,15], certus[:,16], certus[:,17], certus[:,18], certus[:,19], certus[:,20], certus[:,21], certus[:,22]):
    certus_t_matrix = np.matrix(((r00,r01,r02,x), (r10,r11,r12,y), (r20,r21,r22,z), (0,0,0,1)))
    trans_points =  obj_objc_t_matrix * certus_t_matrix
    px.append(trans_points.item(3))
    py.append(trans_points.item(7))
    pz.append(trans_points.item(11))
    
px = np.array(px)
px -= px[0]
py = np.array(py)
py -= py[0]
pz = np.array(pz)
pz -= pz[0]

np.diff(py, 2)

rms = []
for x, y, z in zip(px, py, pz):
    rms.append(math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z,2)))
rms = np.array(rms)

fig = plt.figure(figsize=(PLOT_SIZE[0],PLOT_SIZE[1]))
ax = plt.subplot2grid(GRID, (0,0), colspan=3, rowspan=1)

ax.scatter(certus[:,0], px, color='black', s=0.75)
ax.set_ylabel('$P_X$', fontsize=14)
ax2 = plt.subplot2grid(GRID, (1,0), colspan=3, rowspan=1)

ax2.set_ylabel('$P_Y$', fontsize=14)
ax2.scatter(certus[:,0], py, color='black', s=0.75)
ax3 = plt.subplot2grid(GRID, (2,0), colspan=3, rowspan=1)

ax3.set_ylabel('$P_Z$', fontsize=14)
ax3.scatter(certus[:,0], pz, color='black', s=0.75)

ax_rms =  plt.subplot2grid(GRID, (3,0), colspan=3, rowspan=1)

ax_rms.set_ylabel('$P_{EUC}$', fontsize=14)
ax_rms.set_xlabel('$t[s]$', fontsize=14)
ax_rms.scatter(certus[:,0], rms, color='black', s=0.75)

#ax.set_xlim([0, certus[:,0][-1]])