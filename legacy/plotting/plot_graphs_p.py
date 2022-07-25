import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import matplotlib.patches as mpatches

import itertools
import random
import IPython.display as IPdisplay 
from mpl_toolkits.mplot3d import axes3d, Axes3D 
import sys, getopt
import ntpath

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

BASE_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\'
COLORS = [itertools.cycle(['#ff0000', '#660000']),
          itertools.cycle(['#32CB2E', '#008000']),
          itertools.cycle(['#28ABE3', '#236CF5']),
          itertools.cycle(['#151515', '#696969'])]
GRID = (8,3)
PLOT_SIZE = (9, 11)

def path_leaf(path):
    head, tail = ntpath.split(path)
    return tail or ntpath.basename(head)
    
def setup_plots(placement, columns, rows, dimension, x_limit=[0,60], x_label='', y_label='', z_label=''):
    if dimension == 3:
        ax = plt.subplot2grid(GRID, placement, colspan=columns, rowspan=rows, projection='3d')
    else:
        ax = plt.subplot2grid(GRID, placement, colspan=columns, rowspan=rows)
        ax.set_xlim(x_limit)
    if x_label:
        ax.set_xlabel(x_label, fontsize=14)
    if y_label:
        ax.set_ylabel(y_label, fontsize=14)
    if z_label:
        ax.set_zlabel(z_label, fontsize=14)

    #ax.autoscale()
    return ax

def parse_path_file(input_file):
    path = np.genfromtxt(spatial_path_file, delimiter=',', dtype=None, autostrip=True)
    path['f7'] *= 1000
    path['f8'] *= 1000
    path['f9'] *= 1000
    path['f7'] -= path['f7'][0] # x normalized
    path['f8'] -= path['f8'][0] # y normalized
    path['f9'] -= path['f9'][0] # z normalized
    relative_tool_path = np.argmax([abs(np.diff(path['f7'])),
                            abs(np.diff(path['f8'])),
                            abs(np.diff(path['f9']))], axis=0)
    set_point_color_change = [next(COLORS[3]) for direction in relative_tool_path]
    return path, relative_tool_path, set_point_color_change

def parse_log_file(input_file, relative_tool_path):
    log = np.genfromtxt(log_file, delimiter=',', dtype=np.double, autostrip=True,
                        skip_header=2, usecols=range(0,27))
    log[:,(15,16,17,24,25,26)] = log[:,(15,16,17,24,25,26)]*1000
    t = np.arange(0,log[:,1].size * 0.002, 0.002) #TODO log time
    #t_path = [ for ] 
    log[:,15] -= log[:,15][0] # x normalized
    log[:,16] -= log[:,16][0] # y normalized
    log[:,17] -= log[:,17][0] # z normalized
    
    color_change_locations = abs(np.diff(log[:,0])) > 0
    log_path_colors = [next(COLORS[relative_tool_path[0]])]
    path_point = 0
    t_path = [0]
    t_count = 0.002
    for change in color_change_locations:
        if change: # point satisfaction
            path_point = path_point + 1
            log_path_colors.append(next(COLORS[relative_tool_path[path_point]]))
            t_path.append(t_count)
        log_path_colors.append(log_path_colors[-1])
        t_count = t_count + 0.002
    t_path.append(t_count)
    log_path_colors = itertools.cycle(log_path_colors)  
    return log, log_path_colors, t_path, t

def parse_arguments(argv):
    output_file = None
    input_file = None
    try:
      opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
      print('python plot_graphs.py -i <input_file> -o <output_file>')
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-h':
        print('gcode_parser.py -i <inputfile> -o <outputfile> -dx <Discretization X>, -dy <Discretization Y>, -dz <Discretization Z>, <plot (y/n)>')
        sys.exit()
      elif opt in ("-i", "--ifile"):
        input_file = arg
      elif opt in ("-o", "--ofile"):
        output_file = arg
    return input_file, output_file
    
if __name__=="__main__":
    # load force path and log file from CLI
    
    spatial_path_file = BASE_FILEPATH + 'paths\\load\\mul.csv'
    log_file = BASE_FILEPATH + 'experiments\\mul_v1.csv'
    #spatial_path_file, log_file = parse_arguments(sys.argv[1:])
    if not spatial_path_file or not log_file:
      print('input file fail')
      sys.exit(1)
   
    path, relative_tool_path, set_point_color_change = parse_path_file(spatial_path_file)   
    log, log_path_colors, t_path, t = parse_log_file(log_file, relative_tool_path)
      
    # setup plots
    fig = plt.figure(figsize=(PLOT_SIZE[0],PLOT_SIZE[1]))
    #fx_fy_fz = setup_plots((0, 0), 3, 4, 3, '$F_X [mm]$', '$F_Y [mm]$', '$F_Z [mm]$')
    #fx_t = setup_plots((4, 0), 3, 1, 2, '$t$', '$F_X [mm]$')
    #fy_t = setup_plots((5, 0), 3, 1, 2, '$t$', '$F_Y [mm]$')
    #fz_t = setup_plots((6, 0), 3, 1, 2, '$t$', '$F_Z [mm]$')
    
    #pid_xyz = setup_plots((0, 0), 3, 4, 3, '$PID_X [mm]$', '$PID_Y [mm]$', '$PID_Z [mm]$')
    #pidx_t = setup_plots((4, 0), 3, 1, 2, '$t$', '$PID_x [mm]$')
    #pidy_t = setup_plots((5, 0), 3, 1, 2, '$t$', '$PID_y [mm]$')
    #pidz_t = setup_plots((6, 0), 3, 1, 2, '$t$', '$PID_z [mm]$')
    
    px_py_pz = setup_plots((0, 0), 3, 4, 3, [0,t[-1] + 0.1], '$P_X [mm]$', '$P_Y [mm]$', '$P_Z [mm]$')
    px_t = setup_plots((4, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$P_X [mm]$')
    py_t = setup_plots((5, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$P_Y [mm]$')
    pz_t = setup_plots((6, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$P_Z [mm]$')
    peuc_t = setup_plots((7, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$P_{Euclidian} [mm]$')
    #ff0000', '#660000
    
    # x
    #blue_patch = mpatches.Patch(color='#ff0000', label='$P_{msr}$')
    #black_patch = mpatches.Patch(color='#696969', label='$P_{dsr}$')
    #l = px_py_pz.legend(handles=[blue_patch, black_patch])
    #force_patch = mpatches.Patch(color='#ff0000', label='$F_{msr}$')
    #force_patch2 = mpatches.Patch(color='#696969', label='$F_{dsr}$')   
    #l = fx_fy_fz.legend(handles=[force_patch, force_patch2])   

    # y '#32CB2E', '#008000'

    # chained
    #blue_patch = mpatches.Patch(color='#28ABE3', label='$Pz_{msr}$')
    #red_patch = mpatches.Patch(color='#ff0000', label='$Py_{msr}$')
    #green_patch = mpatches.Patch(color='#32CB2E', label='$Px_{msr}$')
    #black_patch = mpatches.Patch(color='#696969', label='$P_{dsr}$')
    #l = px_py_pz.legend(loc='upper left', handles=[green_patch, red_patch, blue_patch, black_patch])
    #force_patch = mpatches.Patch(color='#28ABE3', label='$F_{msr}$')
    #force_patch2 = mpatches.Patch(color='#696969', label='$F_{dsr}$')   
    #l = fx_fy_fz.legend(handles=[force_patch, force_patch2]) 

    
    # line segments will improve efficiency here
    #######################################
     
    #lc = mc.LineCollection([[(arr2[i][0],arr2[i+1][0]),(arr2[i][1],arr2[i+1][1])] for i in range(len(path) - 1)], colors=COLORS[0][0])
    #fx_t.add_collection(lc)
    
    
    ps = [(ts,x,y,z, pi_x, pi_y, pi_z, fx, fy, fz, db_x, db_y, db_z) for (ts,x,y,z, pi_x, pi_y, pi_z, fx, fy, fz, db_x, db_y, db_z) in zip(t, log[:,15], log[:,16], log[:,17], log[:,24], log[:,25], log[:,26], log[:,6],log[:,7], log[:,8], log[:,0], log[:,1], log[:,2])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z, pi_x, pi_y, pi_z, fx, fy, fz, db_x, db_y, db_z = zip(start, end)
        c = next(log_path_colors)
        cc = next(COLORS[3])
        px_py_pz.plot(x, y, z, color=c)  
        px_t.plot(ts, x, color=c)
        py_t.plot(ts, y, color=c)
        pz_t.plot(ts, z, color=c)
        peuc_t.plot(np.sqrt(np.power(path['f7'], 2) + np.power(path['f8'], 2) + np.power(path['f9'], 2)), color=c)
        
        #pidx_t.plot(ts, pi_x, color=c)
        #pidy_t.plot(ts, pi_y, color=c)
        #pidz_t.plot(ts, pi_z, color=c)
        #fx_t.plot(ts, fx, color=c)
        #fx_t.plot(ts, db_x, color='#7D26CD')
        #fy_t.plot(ts, fy, color=c)
        #fy_t.plot(ts, db_y, color='#7D26CD')
        #fz_t.plot(ts, fz, color=c)
        #fz_t.plot(ts, db_z, color='#7D26CD')
        #fx_fy_fz.plot(fx, fy, fz, color=c)
        #fx_fy_fz.plot(db_x, db_y, db_z, color=cc)
    
    ps = [(ts,x,y,z,fx,fy,fz) for (ts,x,y,z,fx,fy,fz) in zip(t_path,path['f7'], path['f8'], path['f9'] ,path['f1'], path['f2'], path['f3'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z, fx, fy, fz = zip(start, end)
        c = next(COLORS[3])
        px_py_pz.plot(x, y, z, color=c)  
        px_t.plot(ts, x, color=c)
        py_t.plot(ts, y, color=c)
        pz_t.plot(ts, z, color=c)
        peuc_t.plot(ts, np.sqrt(np.power(path['f7'], 2) + np.power(path['f8'], 2) + np.power(path['f9'], 2)), color=c)
        #fx_t.plot(ts, fx, color=c)
        #fy_t.plot(ts, fy, color=c)
        #fz_t.plot(ts, fz, color=c)
        #fx_fy_fz.plot(fx, fy, fz, color=c)
    
        '''
    fx_t.scatter(t, log[:,6], color=log_path_colors, s=1)
    fy_t.scatter(t, log[:,7], color=log_path_colors, s=1)
    fz_t.scatter(t, log[:,8], color=log_path_colors, s=1)
    
    pidx_t.scatter(t, log[:,18], color=log_path_colors, s=1)
    pidy_t.scatter(t, log[:,19], color=log_path_colors, s=1)
    pidz_t.scatter(t, log[:,20], color=log_path_colors, s=1)
    
    px_t.scatter(t, log[:,15], color=log_path_colors, s=1)
    py_t.scatter(t, log[:,16], color=log_path_colors, s=1)
    pz_t.scatter(t, log[:,17], color=log_path_colors, s=1)
    '''
    ####################################################

    
    #plt.savefig('C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\results\\figure.pgf')
    log_filename = path_leaf(log_file)
    RESULTS_PATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\results\\'
    #plt.show()
    fig.savefig(RESULTS_PATH + log_filename[:-4] + '_position.pdf', dpi=150)
    #plt.close()
    
    # ...instead, create an animated gif of all the frames, then display it inline 
    #images = [PIL_Image.open(image) for image in glob.glob('C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\images\\*.png')]
    #file_path_name = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\images\\' + '3D' + '.gif'
    #writeGif(file_path_name, images, duration=0.2)
    #IPdisplay.Image(url=file_path_name)
