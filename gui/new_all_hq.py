import matplotlib as mpl
#mpl.use("pgf")
pgf_with_pdflatex = {
    "pgf.texsystem": "pdflatex",
    "pgf.preamble": [
         r"\usepackage[utf8x]{inputenc}",
         r"\usepackage[T1]{fontenc}",
         r"\usepackage{cmbright}",
         ]
}
mpl.rcParams.update(pgf_with_pdflatex)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import itertools
import sys, getopt
import ntpath
import os

FIGSIZE = (8,10)  # Fx, Fy, Fy, F_{EUC}, Px, Py, Pz, P_{EUC}, PIDx, PIDy, PIDz
FIGSIZE_3D = (4,4)  # 3D(Ax, By, Cz)
FIGSIZE_3D_2D = (8,10)  # 3D(Ax, By, Cz), Ax, By, Cz, D_{EUC}
FIGSIZE_8_2D = (8,10)  # Fx, Fy, Fz, F_{EUC}, Px, Py, Pz, P_{EUC}
FIGSIZE_6_2D = (8,10)  # PIDx, PIDy, PIDz, LWRx, LWRy, LWRz
    
BASE_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\'
LOAD_FILEPATH = BASE_FILEPATH + 'paths\\load\\' 
LOG_FILEPATH = BASE_FILEPATH + 'experiments\\'
RESULTS_FILEPATH = BASE_FILEPATH + 'results\\'
SAMPLE_RATE = 0.002

COLORS = [itertools.cycle(['#ff0000', '#660000']),  # red
          itertools.cycle(['#32CB2E', '#008000']),  # green
          itertools.cycle(['#28ABE3', '#236CF5']),  # blue
          itertools.cycle(['#151515', '#696969'])]  # black/gray

LABELS = {}
LABELS['pid'] = {'x': '$PID_X [mm]$',
                 'y': '$PID_Y [mm]$',
                 'z': '$PID_Z [mm]$',
                 't': '$t\ [s]$'}
LABELS['lwr'] = {'x': '$LWR_X [N]$',
                 'y': '$LWR_Y [N]$',
                 'z': '$LWR_Z [N]$',
                 't': '$t\ [s]$'}
LABELS['force'] = {'x': '$F_X [N]$',
                   'y': '$F_Y [N]$',
                   'z': '$F_Z [N]$',
                   'euc': '$F_{EUC} [N]$',
                   't': '$t\ [s]$'}
LABELS['position'] = {'x': '$P_X [mm]$',
                      'y': '$P_Y [mm]$',
                      'z': '$P_Z [mm]$',
                      'euc': '$P_{EUC} [mm]$',
                      't': '$t\ [s]$'}

LOG_PATH_POINT_COLUMN = 1
LOG_FX_COLUMN = 40
LOG_FY_COLUMN = 44
LOG_FZ_COLUMN = 48
LOG_PIDX_COLUMN = 25
LOG_PIDY_COLUMN = 26
LOG_PIDZ_COLUMN = 27
LOG_PX_COLUMN = 52
LOG_PY_COLUMN = 53
LOG_PZ_COLUMN = 54
LOG_COLS = (LOG_PATH_POINT_COLUMN,LOG_FX_COLUMN, LOG_FY_COLUMN, LOG_FZ_COLUMN,
    LOG_PIDX_COLUMN, LOG_PIDY_COLUMN, LOG_PIDZ_COLUMN,
    LOG_PX_COLUMN, LOG_PY_COLUMN, LOG_PZ_COLUMN)

LOAD_PX_COLUMN = 7
LOAD_PY_COLUMN = 8
LOAD_PZ_COLUMN = 9
LOAD_COLS = (LOAD_PX_COLUMN, LOAD_PY_COLUMN, LOAD_PZ_COLUMN)


def path_leaf(path):
    head, tail = ntpath.split(path)
    return tail or ntpath.basename(head)
    

def style_3d_plot(ax, labels, size_font):
  ax.xaxis._axinfo['label']['space_factor'] = 2.3
  ax.yaxis._axinfo['label']['space_factor'] = 2.3
  ax.zaxis._axinfo['label']['space_factor'] = 1.8
  ax.set_xlabel(x_label, fontsize=size_font)
  ax.set_ylabel(y_label, fontsize=size_font)
  ax.set_zlabel(z_label, fontsize=size_font)


'''
Setup plots for plotting.

Args:
  fig: Figure
  grid: Layout type.
  placement: Grid layout location.
  num_plots: Total number of plots.
  size_font: 13pts
  labels: dict id/name to value. Pass dict to set labels
'''
def setup_plots(fig, dimension=0, num_plots=0, size_font=13,labels=None, raw_labels=None):
  
  # 3D plot with four plots below (A,B,C,EUC)
  if dimension == 4:
    gs1 = plt.GridSpec(2,1, hspace=0.2)
    gs2 = plt.GridSpec(8,1, hspace=0)
    ax1 = fig.add_subplot(gs1[0], projection='3d')
    style_3d_plot(ax1, size_font, labels['x'], labels['y'], labels['z'])
    
    ax = fig.add_subplot(gs2[4])
    lower_axes = [ax]
    for i in range(4, 8):
      if i > 4:
        ax = fig.add_subplot(gs2[i], sharex=lower_axes[0])
      ax.locator_params(axis='y', nbins=5, prune='both')
      ax.margins(x=0,y=0.05)
      lower_axes.append(ax)
    
    for ax in zip(lower_axes, [labels['x'], labels['y'], labels['z'], labels['euc']]):
      ax.label_outer()
      ax.annotate(label, xy=(0.02, 0.5), xytext=(5, 0), rotation=90,
        fontsize=size_font, xycoords=('figure fraction', 'axes fraction'),
        textcoords='offset points', va='center', ha='left')        

    lower_axes[-1].set_xlabel(labels['t'], fontsize=size_font)

    return [ax1] + lower_axes

  # 3D plot
  elif dimension == 3:
    gs = plt.GridSpec(1,1)
    ax = plt.add_subplot(gs[0], projection='3d')
    style_3d_plot(ax, size_font, labels['x'], labels['y'], labels['z'])
    return ax

  # 2D plot
  elif dimension == 2:
    gs = plt.GridSpec(num_plots, 1, hspace=0)
    ax = fig.add_subplot(gs[0])
    axes = [ax]
    for i in range(0, num_plots):
      if i > 0:
        ax = fig.add_subplot(gs[i], sharex=lower_axes[0])
      ax.locator_params(axis='y', nbins=5, prune='both')
      ax.margins(x=0,y=0.05)
      ax.set_ylabel(raw_labels[i + 1])
      axes.append(ax)
    ax.set_xlabel(raw_labels[0], fontsize=size_font)
    return axes


def parse_path_file(input_file):
  path = np.genfromtxt(spatial_path_file, delimiter=',', dtype=np.double,
    autostrip=True, cols=LOAD_COLS)
  # convert m -> mm
  path[:,(0,1,2)] = path[:,(0,1,2)]*1000
  # normalize
  path[:,0] -= path[:,0][0]
  path[:,1] -= path[:,1][0]
  path[:,2] -= path[:,2][0]
  # generate relative tool path
  relative_tool_path = np.argmax([abs(np.diff(path[:,0])),
    abs(np.diff(path[:,1])), abs(np.diff(path[:,2]))], axis=0)
  return path, relative_tool_path


def parse_log_file(input_file, relative_tool_path):
  log = np.genfromtxt(log_file, delimiter=',', dtype=np.double,
      autostrip=True, skip_header=2, cols=LOG_COLS)
  log[:,(0,1,2,3,4,5,6,7,8,9)] = log[:,(0,1,2,3,4,5,6,7,8,9)]*1000
  t = np.arange(0,log[:,1].size * SAMPLE_RATE, SAMPLE_RATE)
  # normalize
  log[:,1] -= log[:,1][0]
  log[:,2] -= log[:,2][0]
  log[:,3] -= log[:,3][0]
    
  color_change_locations = abs(np.diff(log[:,0])) > 0
  log_path_colors = [next(COLORS[relative_tool_path[0]])]
  path_point = 0
  t_path = [0]
  t_count = SAMPLE_RATE
  for change in color_change_locations:
    if change: # point satisfaction
      path_point = path_point + 1
      log_path_colors.append(next(COLORS[relative_tool_path[path_point]]))
      t_path.append(t_count)
    log_path_colors.append(log_path_colors[-1])
    t_count = t_count + SAMPLE_RATE
  t_path.append(t_count)
  log_path_colors = itertools.cycle(log_path_colors)  
  return log, log_path_colors, t_path, t

def parse_arguments(argv):
  output_file = None
  input_file = None
  try:
    opts, args = getopt.getopt(argv,"i:o:",["ifile=","ofile="])
  except getopt.GetoptError:
    print('python plot_graphs.py -i <input_file> -o <output_file>')
    sys.exit(2)
  if len(args) != 2:
    print('-i <input_file> -o <output_file>')
    sys.exit(2)
  for opt, arg in opts:
    if opt in ("-i", "--ifile"):
      input_file = arg
    elif opt in ("-o", "--ofile"):
      output_file = arg
  return input_file, output_file
    

def main():
  fig = plt.figure(figsize=FIGSIZE)
    
  spatial_path_file = 'mover_v2.csv'
  log_file = 'mover_v2_2.csv'
  #spatial_path_file, log_file = parse_arguments(sys.argv[1:])
  plots ='force'      
  if not spatial_path_file or not log_file:
    print('input file fail')
    sys.exit(1)
  plots = plots.split(' ')

  # get data from log files
  path, relative_tool_path, set_point_color_change = parse_path_file(spatial_path_file)   
  log, log_path_colors, t_path, t = parse_log_file(log_file, relative_tool_path)
      
  # setup plots
  if 'force' in plots:
    axes = setup_plots(fig, dimension=4, labels=LABELS['force'])
    axes[0].scatter(log[:,1], log[:,2], log[:,3])
      
  clear = lambda: os.system('cls')
  clear()
    

def plot_positions_forces_pid(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
     # 2D plot
    fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    px_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_X [mm]$')
    py_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Y [mm]$')
    pz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Z [mm]$')
    peuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$P_{EUC} [mm]$')
    fx_t = setup_plots(plot_large[1], (4, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_X [N]$')
    fy_t = setup_plots(plot_large[1], (5, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Y [N]$')
    fz_t = setup_plots(plot_large[1], (6, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Z [N]$')
    feuc_t = setup_plots(plot_large[1], (7, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$F_{EUC} [N]$')
    pidx_t = setup_plots(plot_large[1], (8, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$PID_X [mm]$')
    pidy_t = setup_plots(plot_large[1], (9, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$PID_Y [mm]$')
    pidz_t = setup_plots(plot_large[1], (10, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t[s]$', '$PID_Z [mm]$')
    
    ps = [(ts,x,y,z, fx, fy, fz, pidx, pidy, pidz) for (ts,x,y,z, fx, fy, fz, pidx, pidy, pidz) in zip(t, log[:,40], log[:,44], log[:,48], log[:,25], log[:,26], log[:,27], log[:,52], log[:,53], log[:,54])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z, fx, fy, fz, pidx, pidy, pidz = zip(start, end)
        c = next(log_path_colors)  
        px_t.plot(ts, x, color=c)
        py_t.plot(ts, y, color=c)
        pz_t.plot(ts, z, color=c)
        peuc_t.plot(ts, np.sqrt(np.power(x, 2) + np.power(y, 2) + np.power(z, 2)), color=c)
        fx_t.plot(ts, fx, color=c)
        fy_t.plot(ts, fy, color=c)
        fz_t.plot(ts, fz, color=c)
        feuc_t.plot(ts, np.sqrt(np.power(fx, 2) + np.power(fy, 2) + np.power(fz, 2)), color=c)
        pidx_t.plot(ts, pidx, color=c)
        pidy_t.plot(ts, pidy, color=c)
        pidz_t.plot(ts, pidz, color=c)
      
    ps = [(ts2,x2,y2,z2, fx2, fy2, fz2) for (ts2,x2,y2,z2, fx2, fy2, fz2) in zip(t_path,path['f7'], path['f8'], path['f9'], path['f1'], path['f2'], path['f3'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, x2, y2, z2, fx2, fy2, fz2 = zip(start, end)
        c = next(COLORS[3]) 
        px_t.plot(ts2, x2, color=c)
        py_t.plot(ts2, y2, color=c)
        pz_t.plot(ts2, z2, color=c)
        peuc_t.plot(ts2, np.sqrt(np.power(x2, 2) + np.power(y2, 2) + np.power(z2, 2)), color=c) 
        fx_t.plot(ts2, fx2, color=c)
        fy_t.plot(ts2, fy2, color=c)
        fz_t.plot(ts2, fz2, color=c)
        feuc_t.plot(ts2, np.sqrt(np.power(fx2, 2) + np.power(fy2, 2) + np.power(fz2, 2)), color=c) 
        
    fig.subplots_adjust(hspace=0)
    for ax in [px_t, py_t, pz_t, peuc_t, fx_t, fy_t, fz_t, feuc_t, pidx_t, pidy_t]:
        plt.setp(ax.get_xticklabels(), visible=False)
        
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_plt.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
   
def plot_velocity(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
    
    # 3D plot
    '''
    PLOT_3D = ((8,6),(4,4))
    fig = plt.figure(figsize=(PLOT_3D[0][0],PLOT_3D[0][1]))
    px_py_pz = setup_plots(plot_large[1], (0, 0), 4, 4, 3, [0,t[-1] + 0.1], '$P_X [mm]$', '$P_Y [mm]$', '$P_Z [mm]$')
    ps = [(ts,x,y,z) for (ts,x,y,z) in zip(t, log[:,40], log[:,44], log[:,48])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z = zip(start, end)
        c = next(log_path_colors)
        px_py_pz.plot(x, y, z, color=c)  
      
    ps = [(ts2,x2,y2,z2) for (ts2,x2,y2,z2) in zip(t_path,path['f7'], path['f8'], path['f9'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, x2, y2, z2 = zip(start, end)
        c = next(COLORS[3])
        px_py_pz.plot(x2, y2, z2, color=c)       
    
        fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_position3d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    '''
    # 2D plot
    
    fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    #plt.title(r"Velocity vs. Time", fontsize=18)
    px_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$V_X [mm/s]$')
    py_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$V_Y [mm/s]$')
    pz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$V_Z [mm/s]$')
    peuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$V_{EUC} [mm/s]$')
    
    v_x = np.diff(log[:,40])
    v_y = np.diff(log[:,44])
    v_z = np.diff(log[:,48])
    v_x = np.insert(v_x, 0, 0)
    v_y = np.insert(v_y, 0, 0)
    v_z = np.insert(v_z, 0, 0)
    v_x /= 0.002
    v_y /= 0.002
    v_z /= 0.002
    t += 0.001
    t2 = np.insert(t, 0, 0)
    
    ps = [(ts,x,y,z) for (ts,x,y,z) in zip(t2, v_x, v_y, v_z)]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z = zip(start, end)
        c = next(log_path_colors)  
        px_t.plot(ts, x, color=c)
        py_t.plot(ts, y, color=c)
        pz_t.plot(ts, z, color=c)
        peuc_t.plot(ts, np.sqrt(np.power(x, 2) + np.power(y, 2) + np.power(z, 2)), color=c)
    ''' 
    ps = [(ts2,x2,y2,z2) for (ts2,x2,y2,z2) in zip(t_path,path['f7'], path['f8'], path['f9'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, x2, y2, z2 = zip(start, end)
        c = next(COLORS[3]) 
        px_t.plot(ts2, x2, color=c)
        py_t.plot(ts2, y2, color=c)
        pz_t.plot(ts2, z2, color=c)
        peuc_t.plot(ts2, np.sqrt(np.power(x2, 2) + np.power(y2, 2) + np.power(z2, 2)), color=c) 
    '''
    fig.subplots_adjust(hspace=0)
    for ax in [px_t, py_t, pz_t]:
        plt.setp(ax.get_xticklabels(), visible=False)
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_velocity2d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
def plot_accel(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
    
    # 3D plot
    '''
    PLOT_3D = ((8,6),(4,4))
    fig = plt.figure(figsize=(PLOT_3D[0][0],PLOT_3D[0][1]))
    px_py_pz = setup_plots(plot_large[1], (0, 0), 4, 4, 3, [0,t[-1] + 0.1], '$P_X [mm]$', '$P_Y [mm]$', '$P_Z [mm]$')
    ps = [(ts,x,y,z) for (ts,x,y,z) in zip(t, log[:,40], log[:,44], log[:,48])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z = zip(start, end)
        c = next(log_path_colors)
        px_py_pz.plot(x, y, z, color=c)  
      
    ps = [(ts2,x2,y2,z2) for (ts2,x2,y2,z2) in zip(t_path,path['f7'], path['f8'], path['f9'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, x2, y2, z2 = zip(start, end)
        c = next(COLORS[3])
        px_py_pz.plot(x2, y2, z2, color=c)       
    
        fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_position3d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    '''
    # 2D plot
    
    fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    plt.title(r"Acceleration vs. Time", fontsize=18)
    px_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$A_X [mm^2/s]$')
    py_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$A_Y [mm^2/s]$')
    pz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$A_Z [mm^2/s]$')
    peuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$A_{EUC} [mm]$')
    
    ps = [(ts,x,y,z) for (ts,x,y,z) in zip(t, np.diff(log[:,40]), np.diff(log[:,44]), np.diff(log[:,48]))]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z = zip(start, end)
        c = next(log_path_colors)  
        px_t.plot(ts, x, color=c)
        py_t.plot(ts, y, color=c)
        pz_t.plot(ts, z, color=c)
        peuc_t.plot(ts, np.sqrt(np.power(x, 2) + np.power(y, 2) + np.power(z, 2)), color=c)
    ''' 
    ps = [(ts2,x2,y2,z2) for (ts2,x2,y2,z2) in zip(t_path,path['f7'], path['f8'], path['f9'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, x2, y2, z2 = zip(start, end)
        c = next(COLORS[3]) 
        px_t.plot(ts2, x2, color=c)
        py_t.plot(ts2, y2, color=c)
        pz_t.plot(ts2, z2, color=c)
        peuc_t.plot(ts2, np.sqrt(np.power(x2, 2) + np.power(y2, 2) + np.power(z2, 2)), color=c) 
    '''
    fig.subplots_adjust(hspace=0)
    for ax in [px_t, py_t, pz_t]:
        plt.setp(ax.get_xticklabels(), visible=False)
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_accel2d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
def plot_positions(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
    
    # 3D plot
    PLOT_3D = ((8,6),(4,4))
    fig = plt.figure(figsize=(PLOT_3D[0][0],PLOT_3D[0][1]))
    px_py_pz = setup_plots(plot_large[1], (0, 0), 4, 4, 3, [0,t[-1] + 0.1], '$P_X [mm]$', '$P_Y [mm]$', '$P_Z [mm]$')
    ps = [(ts,x,y,z) for (ts,x,y,z) in zip(t, log[:,40], log[:,44], log[:,48])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z = zip(start, end)
        c = next(log_path_colors)
        px_py_pz.plot(x, y, z, color=c)  
      
    ps = [(ts2,x2,y2,z2) for (ts2,x2,y2,z2) in zip(t_path,path['f7'], path['f8'], path['f9'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, x2, y2, z2 = zip(start, end)
        c = next(COLORS[3])
        px_py_pz.plot(x2, y2, z2, color=c)       
    
    plt.title(r"$\vec P_{DSR}\ vs.\ \vec P_{MSR}$", fontsize=18)
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_position3d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    # 2D plot
    '''
    fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    px_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_X [mm]$')
    py_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Y [mm]$')
    pz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Z [mm]$')
    peuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$P_{EUC} [mm]$')
    
    ps = [(ts,x,y,z) for (ts,x,y,z) in zip(t, log[:,15], log[:,16], log[:,17])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, x, y, z = zip(start, end)
        c = next(log_path_colors)  
        px_t.plot(ts, x, color=c)
        py_t.plot(ts, y, color=c)
        pz_t.plot(ts, z, color=c)
        peuc_t.plot(ts, np.sqrt(np.power(x, 2) + np.power(y, 2) + np.power(z, 2)), color=c)
      
    ps = [(ts2,x2,y2,z2) for (ts2,x2,y2,z2) in zip(t_path,path['f7'], path['f8'], path['f9'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, x2, y2, z2 = zip(start, end)
        c = next(COLORS[3]) 
        px_t.plot(ts2, x2, color=c)
        py_t.plot(ts2, y2, color=c)
        pz_t.plot(ts2, z2, color=c)
        peuc_t.plot(ts2, np.sqrt(np.power(x2, 2) + np.power(y2, 2) + np.power(z2, 2)), color=c) 
    
    fig.subplots_adjust(hspace=0)
    for ax in [px_t, py_t, pz_t]:
        plt.setp(ax.get_xticklabels(), visible=False)
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_position2d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    '''
 
def plot_forces(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
    '''
    # 3D plot
    PLOT_3D = ((8,6),(4,3))
    fig = plt.figure(figsize=(PLOT_3D[0][0],PLOT_3D[0][1]))
    fx_fy_fz = setup_plots(plot_large[1], (0, 0), 3, 4, 3, [0,t[-1] + 0.1], '$F_X [N]$', '$F_Y [N]$', '$F_Z [N]$')
    ps = [(ts,fx,fy,fz) for (ts,fx,fy,fz) in zip(t, log[:,25], log[:,26], log[:,27])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, fx, fy, fz = zip(start, end)
        c = next(log_path_colors)
        fx_fy_fz.plot(fx, fy, fz, color=c)  
      
    ps = [(fx2,fy2,fz2) for (fx2,fy2,fz2) in zip(path['f1'], path['f2'], path['f3'])]
    for start, end in zip(ps[:-1], ps[1:]):
        fx2, fy2, fz2 = zip(start, end)
        c = next(COLORS[3])
        fx_fy_fz.plot(fx2, fy2, fz2, color=c)      
    
    #z = fx_fy_fz.get_zaxis()
    #z.set_major_locator(plt.MaxNLocator(integer=True))
    plt.title(r"$\vec F_{DSR}\ vs.\ \vec F_{MSR}$", fontsize=18)
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_force3d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    '''
    fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    fx_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_X [N]$')
    fy_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Y [N]$')
    fz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Z [N]$')
    feuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$F_{EUC} [N]$')

     
    ps = [(ts,fx,fy,fz) for (ts,fx,fy,fz) in zip(t, log[:,25], log[:,26], log[:,27])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, fx, fy, fz = zip(start, end)
        c = next(log_path_colors)
        fx_t.plot(ts, fx, color=c)
        fy_t.plot(ts, fy, color=c)
        fz_t.plot(ts, fz, color=c)
        feuc_t.plot(ts, np.sqrt(np.power(fx, 2) + np.power(fy, 2) + np.power(fz, 2)), color=c)
      
    ps = [(ts2,fx2,fy2,fz2) for (ts2,fx2,fy2,fz2) in zip(t_path,path['f1'], path['f2'], path['f3'])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts2, fx2, fy2, fz2 = zip(start, end)
        c = next(COLORS[3])
        fx_t.plot(ts2, fx2, color=c)
        fy_t.plot(ts2, fy2, color=c)
        fz_t.plot(ts2, fz2, color=c)
        feuc_t.plot(ts2, np.sqrt(np.power(fx2, 2) + np.power(fy2, 2) + np.power(fz2, 2)), color=c) 
    
    fig.subplots_adjust(hspace=0)
    for ax in [fx_t, fy_t, fz_t]:
        plt.setp(ax.get_xticklabels(), visible=False)
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_force2d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    

def plot_pid(plot_small, t, log, log_path_colors, t_path, path, log_filename, folder):
    fig = plt.figure(figsize=(plot_small[0][0],plot_small[0][1]))
    pidx_t = setup_plots(plot_small[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$PID_X [mm]$')
    pidy_t = setup_plots(plot_small[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$PID_Y [mm]$')
    pidz_t = setup_plots(plot_small[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$PID_Z [mm]$')

      
    ps = [(ts,pidx, pidy, pidz) for (ts,pidx, pidy, pidz) in zip(t, log[:,52], log[:,53], log[:,54])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, pidx, pidy, pidz = zip(start, end)
        c = next(log_path_colors)
        pidx_t.plot(ts, pidx, color=c)
        pidy_t.plot(ts, pidy, color=c)
        pidz_t.plot(ts, pidz, color=c)
        
    fig.subplots_adjust(hspace=0)
    for ax in [pidx_t, pidy_t]:
        plt.setp(ax.get_xticklabels(), visible=False)
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_pid.pdf', dpi=150, bbox_inches='tight') 
    plt.close(fig)

def plot_lwr(plot_small, t, log, log_path_colors, t_path, path, log_filename, folder):
    fig = plt.figure(figsize=(plot_small[0][0],plot_small[0][1]))
    pidx_t = setup_plots(plot_small[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$LWR_{EXT} [N]$')
    pidy_t = setup_plots(plot_small[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$LWR_{EXT} [N]$')
    pidz_t = setup_plots(plot_small[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$LWR_{EXT} [N]$')

      
    ps = [(ts,lwrx, lwry, lwrz) for (ts,lwrx, lwry, lwrz) in zip(t, log[:,43], log[:,44], log[:,45])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, pidx, pidy, pidz = zip(start, end)
        c = next(log_path_colors)
        pidx_t.plot(ts, pidx, color=c)
        pidy_t.plot(ts, pidy, color=c)
        pidz_t.plot(ts, pidz, color=c)

    fig.subplots_adjust(hspace=0)
    for ax in [pidx_t, pidy_t]:
        plt.setp(ax.get_xticklabels(), visible=False)    
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_lwrext.pdf', dpi=150, bbox_inches='tight') 
    plt.close(fig)

def plot_err(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
    fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    fx_t = setup_plots(plot_large[1], (4, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_X [N]$')
    fy_t = setup_plots(plot_large[1], (5, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Y [N]$')
    fz_t = setup_plots(plot_large[1], (6, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Z [N]$')
    feuc_t = setup_plots(plot_large[1], (7, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$F_{EUC} [N]$')
    px_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_X [mm]$')
    py_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Y [mm]$')
    pz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Z [mm]$')
    peuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_{EUC} [mm]$')

    fx_err = abs(log[:,25] - np.interp(t, t_path, path['f1']))
    fy_err = abs(log[:,26] - np.interp(t, t_path, path['f2']))
    fz_err = abs(log[:,27] - np.interp(t, t_path, path['f3']))
    f_rms = np.sqrt(np.power(fx_err, 2) + np.power(fy_err, 2) + np.power(fz_err, 2))
    px_err = abs(log[:,40] - np.interp(t, t_path, path['f7']))
    py_err = abs(log[:,44] - np.interp(t, t_path, path['f8']))
    pz_err = abs(log[:,48] - np.interp(t, t_path, path['f9']))
    p_rms = np.sqrt(np.power(px_err, 2) + np.power(py_err, 2) + np.power(pz_err, 2))
    
    print('Euclidean force error: average (%.3fN), standard deviation (%.3fN), and peak (%.3fN). Euclidean position error: average (%.3fmm), standard deviation (%.3fmm), and peak (%.3fmm)' % (np.average(f_rms), np.std(f_rms), np.max(f_rms), np.average(p_rms), np.std(p_rms), np.max(p_rms)))
    
    ps = [(ts,fx, fy, fz, f, px, py, pz, p) for (ts,fx, fy, fz, f, px, py, pz, p) in zip(t, fx_err, fy_err, fz_err, f_rms, px_err, py_err, pz_err, p_rms)]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, fx, fy, fz, f, px, py, pz, p = zip(start, end)
        c = next(log_path_colors)    
        fx_t.plot(ts, fx, color=c)
        fy_t.plot(ts, fy, color=c)
        fz_t.plot(ts, fz, color=c)
        feuc_t.plot(ts, f, color=c)
        px_t.plot(ts, px, color=c)
        py_t.plot(ts, py, color=c)
        pz_t.plot(ts, pz, color=c)
        peuc_t.plot(ts, p, color=c)
 
    fig.subplots_adjust(hspace=0)
    for ax in [fx_t,fy_t,fz_t,peuc_t,px_t,py_t,pz_t]:
        plt.setp(ax.get_xticklabels(), visible=False)   
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_error.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
if __name__=="__main__":
  main()
