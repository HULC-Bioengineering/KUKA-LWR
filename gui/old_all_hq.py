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

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import itertools
import sys, getopt
import ntpath
import os

BASE_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\'
RESULTS_PATH = 'C:\\Users\\HMMS\\Google Drive\\Masters\\Results\\huge\\'
COLORS = [itertools.cycle(['#ff0000', '#660000']),
          itertools.cycle(['#32CB2E', '#008000']),
          itertools.cycle(['#28ABE3', '#236CF5']),
          itertools.cycle(['#151515', '#696969'])]

def path_leaf(path):
    head, tail = ntpath.split(path)
    return tail or ntpath.basename(head)
    
def setup_plots(grid, placement, columns, rows, dimension, x_limit=[0,60], x_label='', y_label='', z_label='', share=None):
    if dimension == 3:
        ax = plt.subplot2grid(grid, placement, colspan=columns, rowspan=rows, projection='3d')
    else:
        if share:
            ax = plt.subplot2grid(grid, placement, colspan=columns, rowspan=rows, sharex=share)
        else:
            ax = plt.subplot2grid(grid, placement, colspan=columns, rowspan=rows)
        ax.margins(x=0, y=0.05)        
        ax.locator_params(axis='y', nbins=6, prune='both')
    if x_label:
        ax.set_xlabel(x_label, fontsize=13)
    if y_label:
        #ax.set_ylabel(y_label, fontsize=14)
        ax.annotate(y_label, xy=(0.02, 0.5), xytext=(5, 0), rotation=90, fontsize=13,
            xycoords=('figure fraction', 'axes fraction'),
            textcoords='offset points', va='center', ha='left')        
    if z_label:
        ax.set_zlabel(z_label, fontsize=13)

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
    plots = None
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
      elif opt in ("-p", "--plots"):
        plots = arg
    return input_file, output_file, plots
    
    
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
    
    ps = [(ts,x,y,z, fx, fy, fz, pidx, pidy, pidz) for (ts,x,y,z, fx, fy, fz, pidx, pidy, pidz) in zip(t, log[:,15], log[:,16], log[:,17], log[:,6], log[:,7], log[:,8], log[:,24], log[:,25], log[:,26])]
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
        
    #ax.axvspan(8, 14, alpha=0.75, color='gray')
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_plt.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
def plot_positions(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
    
    # 3D plot
    PLOT_3D = ((8,6),(4,3))
    fig = plt.figure(figsize=(PLOT_3D[0][0],PLOT_3D[0][1]))
    px_py_pz = setup_plots(plot_large[1], (0, 0), 3, 4, 3, [0,t[-1] + 0.1], '$P_X [mm]$', '$P_Y [mm]$', '$P_Z [mm]$')
    ps = [(ts,x,y,z) for (ts,x,y,z) in zip(t, log[:,15], log[:,16], log[:,17])]
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
    
    # 2D plot
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
 
def plot_forces(plot_large, t, log, log_path_colors, t_path, path, log_filename, folder):
    
    # 3D plot
    PLOT_3D = ((8,6),(4,3))
    fig = plt.figure(figsize=(PLOT_3D[0][0],PLOT_3D[0][1]))
    fx_fy_fz = setup_plots(plot_large[1], (0, 0), 3, 4, 3, [0,t[-1] + 0.1], '$F_X [N]$', '$F_Y [N]$', '$F_Z [N]$')
    ps = [(ts,fx,fy,fz) for (ts,fx,fy,fz) in zip(t, log[:,6], log[:,7], log[:,8])]
    for start, end in zip(ps[:-1], ps[1:]):
        ts, fx, fy, fz = zip(start, end)
        c = next(log_path_colors)
        fx_fy_fz.plot(fx, fy, fz, color=c)  
      
    ps = [(fx2,fy2,fz2) for (fx2,fy2,fz2) in zip(path['f1'], path['f2'], path['f3'])]
    for start, end in zip(ps[:-1], ps[1:]):
        fx2, fy2, fz2 = zip(start, end)
        c = next(COLORS[3])
        fx_fy_fz.plot(fx2, fy2, fz2, color=c)      
    
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_force3d.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    fx_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_X [N]$')
    fy_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Y [N]$')
    fz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Z [N]$')
    feuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$F_{EUC} [N]$')

      
    ps = [(ts,fx,fy,fz) for (ts,fx,fy,fz) in zip(t, log[:,6], log[:,7], log[:,8])]
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

      
    ps = [(ts,pidx, pidy, pidz) for (ts,pidx, pidy, pidz) in zip(t, log[:,24], log[:,25], log[:,26])]
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
    #fig = plt.figure(figsize=(plot_large[0][0],plot_large[0][1]))
    #fx_t = setup_plots(plot_large[1], (0, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_X [N]$')
    #fy_t = setup_plots(plot_large[1], (1, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Y [N]$')
    #fz_t = setup_plots(plot_large[1], (2, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Z [N]$')
    #feuc_t = setup_plots(plot_large[1], (3, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$F_{EUC} [N]$')
    #px_t = setup_plots(plot_large[1], (4, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_X [mm]$')
    #py_t = setup_plots(plot_large[1], (5, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Y [mm]$')
    #pz_t = setup_plots(plot_large[1], (6, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$P_Z [mm]$')
    #peuc_t = setup_plots(plot_large[1], (7, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$P_{EUC} [mm]$')

    fx_err = abs(log[:,6] - np.interp(t, t_path, path['f1']))
    fy_err = abs(log[:,7] - np.interp(t, t_path, path['f2']))
    fz_err = abs(log[:,8] - np.interp(t, t_path, path['f3']))
    f_rms = np.sqrt(np.power(fx_err, 2) + np.power(fy_err, 2) + np.power(fz_err, 2))
    px_err = abs(log[:,15] - np.interp(t, t_path, path['f7']))
    py_err = abs(log[:,16] - np.interp(t, t_path, path['f8']))
    pz_err = abs(log[:,17] - np.interp(t, t_path, path['f9']))
    p_rms = np.sqrt(np.power(px_err, 2) + np.power(py_err, 2) + np.power(pz_err, 2))
    
    #print(log_filename)
    #print('f_rms')
    #print(np.average(f_rms))
    #print(np.std(f_rms))
    #print(np.max(f_rms))
    
    #print('p_rms')
    #print(np.average(p_rms))
    #print(np.std(p_rms))
    #print(np.max(p_rms))   
    
    #print('Euclidean force error: average (%.3fN), standard deviation (%.3fN), and peak (%.3fN). Euclidean position error: average (%.3fmm), standard deviation (%.3fmm), and peak (%.3fmm)' % (np.average(f_rms), np.std(f_rms), np.max(f_rms), np.average(p_rms), np.std(p_rms), np.max(p_rms)))
    print('%.2f & %.2f & %.2f & %.2f & %.2f & %.2f' % (np.average(f_rms), np.std(f_rms), np.max(f_rms), np.average(p_rms), np.std(p_rms), np.max(p_rms)))    
    '''
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
    for ax in [fx_t,fy_t,fz_t,feuc_t,px_t,py_t,pz_t]:
        plt.setp(ax.get_xticklabels(), visible=False)   
    fig.savefig(RESULTS_PATH + folder + log_filename[:-4] + '_error.pdf', dpi=150, bbox_inches='tight')
    plt.close(fig)
    '''
if __name__=="__main__":
    # load force path and log file from CLI
    PLOT_LARGE = ((9, 7), (4,3))  # position, force, error
    PLOT_3D = ((6,6),(4,3))
    PLOT_ERR = ((9,11), (8,3))
    PLOT_SMALL = ((7, 5), (3,3))  # pid, lwr
    PLOT_SUM = ((9,14),(11,3))
    
    # X
    
    FOLDER = ''
    spatial_path_file = RESULTS_PATH + FOLDER + 'multi_final_v20.csv'
    log_files = ['multi_final_v2_3.csv',
                 #'loop_5_y_15_v1_1.csv',
                 #'loop_5_z_10_v1_1.csv'
    ]    
    
    '''
    FOLDER = 'x\\'
    spatial_path_file = RESULTS_PATH + FOLDER + 'loop_x_10_load.csv'    
    log_files = ['loop_x_10_fx.csv',
                 'loop_x_10_fy.csv',
                 'loop_x_10_fz.csv',
                 'loop_x_10_lc_fx.csv',
                 'loop_x_10_lc_fy.csv',
                 'loop_x_10_lc_fz.csv'
                 ]
    
    '''
    # Y
    '''
    FOLDER = 'y\\'
    spatial_path_file = RESULTS_PATH + FOLDER + 'loop_y_15_load.csv'    
    log_files = ['loop_y_15_fx.csv',
                 'loop_y_15_fy.csv',
                 'loop_y_15_fz.csv',
                 'loop_y_15_lc_fx.csv',
                 'loop_y_15_lc_fy.csv',
                 'loop_y_15_lc_fz.csv'
                ]
              
     # Z
    
    FOLDER = 'z\\'
    spatial_path_file = RESULTS_PATH + FOLDER + 'loop_z_10_load.csv'    
    log_files = ['loop_z_10_fx.csv',
                 'loop_z_10_fy.csv',
                 'loop_z_10_fz.csv',
                 'loop_z_10_lc_fx.csv',
                 'loop_z_10_lc_fy.csv',
                 'loop_z_10_lc_fz.csv'
                 ]            
    '''          
    for log_file in log_files:
      log_file = RESULTS_PATH + FOLDER + log_file
      #plots = 'p f pid err a'     
      plots ='err'      
      #print(log_file)      
      #spatial_path_file, log_file, plots = parse_arguments(sys.argv[1:])
      if not spatial_path_file or not log_file:
        print('input file fail')
        sys.exit(1)
      plots = plots.split(' ')
      path, relative_tool_path, set_point_color_change = parse_path_file(spatial_path_file)   
      log, log_path_colors, t_path, t = parse_log_file(log_file, relative_tool_path)
      
    
    # setup plots
      log_filename = path_leaf(log_file)
      if 'a' in plots:
        plot_positions_forces_pid(PLOT_SUM, t, log, log_path_colors, t_path, path, log_filename, FOLDER)
      if 'p' in plots:
        plot_positions(PLOT_LARGE, t, log, log_path_colors, t_path, path, log_filename, FOLDER)
      if 'f' in plots:
        plot_forces(PLOT_LARGE, t, log, log_path_colors, t_path, path, log_filename, FOLDER)        
      if 'pid' in plots:
        plot_pid(PLOT_SMALL, t, log, log_path_colors, t_path, path, log_filename, FOLDER)
      if 'err' in plots:
        plot_err(PLOT_ERR, t, log, log_path_colors, t_path, path, log_filename, FOLDER)
      if 'lwr' in plots:
        plot_lwr(PLOT_SMALL, t, log, log_path_colors, t_path, path, log_filename, FOLDER)
          
      
    clear = lambda: os.system('cls')
    clear()
    #fx_fy_fz = setup_plots((0, 0), 3, 4, 3, '$F_X [mm]$', '$F_Y [mm]$', '$F_Z [mm]$')
    #fx_t = setup_plots((4, 0), 3, 1, 2, '$t$', '$F_X [mm]$')
    #fy_t = setup_plots((5, 0), 3, 1, 2, '$t$', '$F_Y [mm]$')
    #fz_t = setup_plots((6, 0), 3, 1, 2, '$t$', '$F_Z [mm]$')
    
    #pid_xyz = setup_plots((0, 0), 3, 4, 3, '$PID_X [mm]$', '$PID_Y [mm]$', '$PID_Z [mm]$')
    #pidx_t = setup_plots((4, 0), 3, 1, 2, '$t$', '$PID_x [mm]$')
    #pidy_t = setup_plots((5, 0), 3, 1, 2, '$t$', '$PID_y [mm]$')
    #pidz_t = setup_plots((6, 0), 3, 1, 2, '$t$', '$PID_z [mm]$')
    

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
    
    '''
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
    ####################################################

    
    #plt.savefig('C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\results\\figure.pgf')

    #plt.show()
    #fig.savefig(RESULTS_PATH + log_filename[:-4] + '_position.pdf', dpi=150)
    #plt.close()
    
    # ...instead, create an animated gif of all the frames, then display it inline 
    #images = [PIL_Image.open(image) for image in glob.glob('C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\images\\*.png')]
    #file_path_name = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\images\\' + '3D' + '.gif'
    #writeGif(file_path_name, images, duration=0.2)
    #IPdisplay.Image(url=file_path_name)
