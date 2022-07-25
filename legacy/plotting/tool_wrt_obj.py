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

BASE_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\'
GRID = (4,3)
PLOT_SIZE = (6,7)

# static transformations
T_OBJC_WRT_OBJ = inv(np.matrix(((-0.044010,-0.996970,-0.064003,-62.576464),
                                   (-0.451635,0.076999,-0.888866,0.261632),
                                   (0.891117,-0.010213,-0.453655,16.894884),
                                   (0,0,0,1))))
T_BASE_WRT_BASEC = np.matrix(((-0.012754,0.998013,-0.061690,-120.933891),
                              (0.999912,0.012958,0.002909,767.126146),
                              (0.003703,-0.061647,-0.998090,14.715575),
                              (0,0,0,1)))

def parse_transform_path_from_log(filename, columns):
    path = []
    t_basec_wrt_objc = np.genfromtxt(filename, delimiter=',',
                                     dtype=np.double, autostrip=True,
                                     skip_header=5, usecols=columns) 
    for t00, t01, t02, t10, t11, t12, t20, t21, t22, t03, t13, t23 in zip(t_basec_wrt_objc[:,0], t_basec_wrt_objc[:,1], t_basec_wrt_objc[:,2], t_basec_wrt_objc[:,3], t_basec_wrt_objc[:,4], t_basec_wrt_objc[:,5], t_basec_wrt_objc[:,6], t_basec_wrt_objc[:,7], t_basec_wrt_objc[:,8], t_basec_wrt_objc[:,9], t_basec_wrt_objc[:,10], t_basec_wrt_objc[:,11]):
        path.append(np.matrix((t00, t01, t02, t03), (t10, t11, t12, t13), (t20, t21, t22, t23), (0, 0, 0, 1)))    
    return path
  
def align_certus_encoder_time(certus, encoder):
    pass
  
def compute_t_toolenc_obj(t_toolenc_base, t_basec_objc, certus_sample_frequency, encoder_sample_frequency):
    pass

def compute_p_toolc_obj(t_toolc_objc):
    position_path = []
    for t in t_toolc_objc:
        tmp =  T_OBJC_WRT_OBJ * certus_t_matrix
        position_path.append(np.array([tmp.item(3), tmp.item(7), tmp.item(11)]))
    return position_path
    
def plot(t_toolc_obj, t_toolenc_obj, certus_sample_frequency):
    pass

def main():
    certus_sample_frequency = 250  # hz
    encoder_sample_frequency = 500  # hz
    encoder_file = 'final_air'
    certus_file = 'final_air_optical'
    
    encoder_filename = BASE_FILEPATH + encoder_file + '.csv'
    certus_filename = BASE_FILEPATH + certus_file + '.csv'

    ###########
    # parse logs
    ###########

    #  {_{tool.enc}^{base}}T
    t_toolenc_base = parse_transform_path_from_log(encoder_filename, (37,38,39,40,41,42,43,44,45,46,47,48))

    #  {_{base.c}^{obj.c}}T
    t_basec_objc = parse_transform_path_from_log(certus_filename, (27,28,29,30,31,32,33,34,35,36,37,38))    
    
    #  {_{tool.c}^{obj.c}}T
    t_toolc_objc = parse_transform_path_from_log(certus_filename, (14,15,16,17,18,19,20,21,22,23,24,25)) 

    ###########
    # sync time between certus and encoder (reduce samples by 1/2)
    ###########    
    start_enc, start_optical = align_certus_encoder_time(t_toolenc_base, t_toolc_objc)
    t_toolc_objc = t_toolc_objc[start_optical:]    
    t_toolenc_base = t_toolenc_base[start_enc:]   
    t_basec_objc = t_basec_objc[start_optical:]

    ###########
    # compute transformation equation(s)
    ###########      
    p_toolc_obj = compute_p_toolc_obj(t_toolc_objc)
    p_toolenc_obj = compute_t_toolenc_obj(t_toolenc_base, t_basec_objc, certus_sample_frequency, encoder_sample_frequency)
 

    
    ###########
    # plot results
    ###########  
    plot(p_toolc_obj, p_toolenc_obj, certus_sample_frequency)
    
    
if __name__ == "__main__":
    main()


             
certus[:,0] *= 0.002


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

rms = []
for x, y, z in zip(px, py, pz):
    rms.append(math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z,2)))
rms = np.array(rms)

fig = plt.figure(figsize=(PLOT_SIZE[0],PLOT_SIZE[1]))
ax = plt.subplot2grid(GRID, (0,0), colspan=3, rowspan=1)
ax.set_xlim([0, certus[:,0][-1]])
ax.scatter(certus[:,0], px, color='black', s=0.75)
ax.set_ylabel('$P_X$', fontsize=14)
ax2 = plt.subplot2grid(GRID, (1,0), colspan=3, rowspan=1)
ax2.set_xlim([0, certus[:,0][-1]])
ax2.set_ylabel('$P_Y$', fontsize=14)
ax2.scatter(certus[:,0], py, color='black', s=0.75)
ax3 = plt.subplot2grid(GRID, (2,0), colspan=3, rowspan=1)
ax3.set_xlim([0, certus[:,0][-1]])
ax3.set_ylabel('$P_Z$', fontsize=14)
ax3.scatter(certus[:,0], pz, color='black', s=0.75)

ax_rms =  plt.subplot2grid(GRID, (3,0), colspan=3, rowspan=1)
ax_rms.set_xlim([0, certus[:,0][-1]])
ax_rms.set_ylabel('$P_{EUC}$', fontsize=14)
ax_rms.set_xlabel('$t[s]$', fontsize=14)
ax_rms.scatter(certus[:,0], rms, color='black', s=0.75)