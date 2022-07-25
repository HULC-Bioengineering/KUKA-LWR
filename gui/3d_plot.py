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
import itertools
import sys, getopt
import ntpath

BASE_FILEPATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\'
RESULTS_PATH = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\results\\'
COLORS = [itertools.cycle(['#ff0000', '#660000']),
          itertools.cycle(['#32CB2E', '#008000']),
          itertools.cycle(['#28ABE3', '#236CF5']),
          itertools.cycle(['#151515', '#696969'])]
GRID = (4,4)
PLOT_SIZE = (4,4)

def path_leaf(path):
    head, tail = ntpath.split(path)
    return tail or ntpath.basename(head)
    
def setup_plots(placement, columns, rows, dimension, x_limit=[0,60], x_label='', y_label='', z_label=''):
    if dimension == 3:
        ax = plt.subplot2grid(GRID, placement, colspan=columns, rowspan=rows, projection='3d')
    else:
        ax = plt.subplot2grid(GRID, placement, colspan=columns, rowspan=rows)
        ax.set_xlim(x_limit)
        ax.margins(x=0, y=0.05)   
        ax.locator_params(axis='y', nbins=5, prune='both')

    if x_label:
        ax.set_xlabel(x_label, fontsize=13)
    if y_label:
        ax.set_ylabel(y_label, fontsize=13)
        #ax.annotate(y_label, xy=(0.02, 0.5), xytext=(5, 0), rotation=90, fontsize=13,
        #    xycoords=('figure fraction', 'axes fraction'),
        #    textcoords='offset points', va='center', ha='left')        
    if z_label:
        ax.set_zlabel(z_label, fontsize=13)
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
                        skip_header=2)
    log[:,(40,44,48,52,53,54)] = log[:,(40,44,48,52,53,54)]*1000
    t = np.arange(0,log[:,1].size * 0.002, 0.002) #TODO log time
    #t_path = [ for ] 
    log[:,40] -= log[:,40][0] # x normalized
    log[:,44] -= log[:,44][0] # y normalized
    log[:,48] -= log[:,48][0] # z normalized
    
    color_change_locations = abs(np.diff(log[:,1])) > 0
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
    #log_path_colors = itertools.cycle(log_path_colors)  
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
    
#    spatial_path_file = BASE_FILEPATH + 'results\\poke\\move\\loop_10_y_15_load.csv'
#    log_file = BASE_FILEPATH + 'results\\poke\\move\\loop_10_y_15_plat_move.csv'
    spatial_path_file = BASE_FILEPATH + 'paths\\load\\multi_final_v2.csv'
    log_file = BASE_FILEPATH + 'experiments\\multi_final_v2_record.csv'
    log_filename = path_leaf(log_file)
    #spatial_path_file, log_file = parse_arguments(sys.argv[1:])n
    if not spatial_path_file or not log_file:
      print('input file fail')
      sys.exit(1)
   
    path, relative_tool_path, set_point_color_change = parse_path_file(spatial_path_file)   
    log, log_path_colors, t_path, t = parse_log_file(log_file, relative_tool_path)
    
    
    # setup plots
    fig= plt.figure()
    #ax = fig.add_subplot(111,projection='3d')

    

    ax = setup_plots((0, 0), 4, 4, 3, [0,t[-1] + 0.1], '$F_X [N]$', '$F_Y [N]$', '$F_Z [N]$')
    ax.xaxis._axinfo['label']['space_factor'] = 2.3
    ax.yaxis._axinfo['label']['space_factor'] = 2.3
    ax.zaxis._axinfo['label']['space_factor'] = 1.8
    ax.set_xlabel('$P_X [mm]$', fontsize=13)
    ax.set_ylabel('$P_Y [mm]$', fontsize=13)
    ax.set_zlabel('$P_Z [mm]$', fontsize=13)    
    #lwrx_t = setup_plots((11, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$LWR_x [N]$')
    #lwry_t = setup_plots((12, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$LWR_y [N]$')
    #lwrz_t = setup_plots((13, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$LWR_z [N]$')
    #toolx_t = setup_plots((14, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_X [N]$')
    #tooly_t = setup_plots((15, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Y [N]$')
    #toolz_t = setup_plots((16, 0), 3, 1, 2, [0,t[-1] + 0.1], '', '$F_Z [N]$')
    #tooleuc_t = setup_plots((17, 0), 3, 1, 2, [0,t[-1] + 0.1], '$t [s]$', '$F_{EUC} [N]$')
    '''
    fx_t.scatter(t, log[:,25], color=log_path_colors, s=1)
    fy_t.scatter(t, log[:,26], color=log_path_colors, s=1)
    fz_t.scatter(t, log[:,27], color=log_path_colors, s=1)
    f_euc.scatter(t, np.sqrt(np.power(log[:,25], 2) + np.power(log[:,26], 2) + np.power(log[:,27], 2)), color=log_path_colors, s=1)
    
    pidx_t.scatter(t, log[:,52], color=log_path_colors, s=1)
    pidy_t.scatter(t, log[:,53], color=log_path_colors, s=1)
    pidz_t.scatter(t, log[:,54], color=log_path_colors, s=1)
    
    px_t.scatter(t, log[:,40], color=log_path_colors, s=1)
    py_t.scatter(t, log[:,44], color=log_path_colors, s=1)
    pz_t.scatter(t, log[:,48], color=log_path_colors, s=1)
    p_euc.scatter(t, np.sqrt(np.power(log[:,40], 2) + np.power(log[:,44], 2) + np.power(log[:,48], 2)), color=log_path_colors, s=1)
    '''
    #lwrx_t.scatter(t, log[:,31], color=log_path_colors, s=1)
    #lwry_t.scatter(t, log[:,32], color=log_path_colors, s=1)
    #lwrz_t.scatter(t, log[:,33], color=log_path_colors, s=1)
    
    #toolx_t.scatter(t, log[:,31] - log[:,25], color=log_path_colors, s=1)
    #tooly_t.scatter(t, log[:,32] - log[:,26], color=log_path_colors, s=1)
    #toolz_t.scatter(t, log[:,33] - log[:,27], color=log_path_colors, s=1)
    #tooleuc_t.scatter(t, np.sqrt(np.power(log[:,31] - log[:,25], 2) + np.power(log[:,32] - log[:,26], 2) + np.power(log[:,33] - log[:,27], 2)), color=log_path_colors, s=1)
    
    ax.scatter(log[:,40], log[:,44], log[:,48], facecolors=log_path_colors)
    ax.plot(path['f7'], path['f8'], path['f9'], color='black')
    
    '''
    px_t.plot(t_path, path['f7'], color='gray')
    py_t.plot(t_path, path['f8'], color='gray')
    pz_t.plot(t_path, path['f9'], color='gray')
    p_euc.plot(t_path, np.sqrt(np.power(path['f7'], 2) + np.power(path['f8'], 2) + np.power(path['f9'], 2)), color='gray')
    fx_t.plot(t_path, path['f1'], color='gray')
    fy_t.plot(t_path, path['f2'], color='gray')
    fz_t.plot(t_path, path['f3'], color='gray')
    f_euc.plot(t_path, np.sqrt(np.power(path['f1'], 2) + np.power(path['f2'], 2) + np.power(path['f3'], 2)), color='gray')
    '''
    #fig.subplots_adjust(hspace=0)
    
    #for ax in [px_t, py_t, pz_t, p_euc, fx_t, fy_t, fz_t, f_euc, pidx_t, pidy_t]:
    #    plt.setp(ax.get_xticklabels(), visible=False)
        
    #plt.show()
    fig.savefig(RESULTS_PATH + log_filename[:-4] + '_dr2t3_3d.pdf', dpi=150,  bbox_inches='tight')
    #plt.close()
