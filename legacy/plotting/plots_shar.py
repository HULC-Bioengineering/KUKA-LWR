import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import itertools, random, glob, os
import matplotlib.cm as cm, IPython.display as IPdisplay 
from mpl_toolkits.mplot3d import Axes3D
from pylab import *

#####################getting data##############################

filename1 = input('Enter first file location: ')
filename2 = input('Enter second file location: ')
if os.path.isfile(filename1) and os.path.isfile(filename2):
    
    input_data = np.genfromtxt(filename1, delimiter=',', dtype = np.float64)
    log_data = np.genfromtxt(filename2, skiprows = 2, delimiter=',',dtype = np.float64) 
    
    #'E:input_path.csv' these are where the file are stored on my computer
    #'E:log.csv'
################################################################
    
    t = []
    tChange = []  
    count = 0 
    count1 = 0
    colorChange = 0
    i = 0.0
        
    length_input = len(input_data[:,0])
    length_log = len(log_data[:,0])  
    comden1 = 6/length_input
    comden2 = 6/length_log
   
    while count1 < length_input:
        t.append(i)    
        i += comden1
        count1 += 1
        
###################from the first chart#########################  
    
    x1 = np.round(input_data[:,7] * 1000)
    y1 = np.round(input_data[:,8] * 1000)
    z1 = np.round(input_data[:,9] * 1000)
    
    Fx1 = np.round(input_data[:,1], 2)
    Fy1 = np.round(input_data[:,2], 2)
    Fz1 = np.round(input_data[:,3], 2)
    
    #normalize
    Px1 = [a - x1[0] for a in x1]
    Py1 = [a - y1[0] for a in y1]
    Pz1 = [a - z1[0] for a in z1]
    
###############from the second chart############################
    
    x2 = np.round(log_data[:,15]* 1000, 2) #msr data
    y2 = np.round(log_data[:,16]* 1000, 2) #msr data
    z2 = np.round(log_data[:,17]* 1000, 2) #msr data
    
    Fx2 = np.round(log_data[:,6], 2)
    Fy2 = np.round(log_data[:,7], 2)
    Fz2 = np.round(log_data[:,8], 2) 

    dx = log_data[:,21]    
    
    Deadx = np.round(log_data[:,0], 2)
    Deady = np.round(log_data[:,1], 2)
    Deadz = np.round(log_data[:,2], 2)
    
    Pidx = np.round(log_data[:,18]*1000, 2)
    Pidy = np.round(log_data[:,19]*1000, 2)
    Pidz = np.round(log_data[:,20]*1000, 2)
    
    #normalize
    Px2 = [a - x2[0] for a in x2]
    Py2 = [a - y2[0] for a in y2]
    Pz2 = [a - z2[0] for a in z2]

#########################set colours############################
    tChange.append(0)
    while count < length_log-1: 
        if np.absolute(Pidx[count + 1]) - np.absolute(Pidx[count]) > 0.05 or np.absolute(Pidx[count + 1]) - np.absolute(Pidx[count]) < -0.05:
            if np.absolute(dx[count + 1]) - np.absolute(dx[count]):
                colorChange += 1
                tChange.append(count + 2)
        count += 1
    
    tChange.append(length_log)
    
    t1 = [[] for j in range (len(tChange) - 1)]    
    i = 0   
    while i < len(tChange) - 1:
        for n in range (tChange[i], tChange[i + 1]):
            t1[i].append(n*comden2)
        i += 1
    
    n = 0    
    Color = np.empty(colorChange + 1, dtype = 'object')
    while n <= colorChange:
        Color[n] = 'green'
        Color[n + 1] = 'red'
        Color[n + 2] = 'green'
        Color[n + 3] = 'red'
        Color[n + 4] = 'green'
        Color[n + 5] = 'blue'
        n += 6
    
#####################setting variables#########################   
    Px_2 = [[] for k in range(len(tChange) - 1)]
    Py_2 = [[] for k in range(len(tChange) - 1)]
    Pz_2 = [[] for k in range(len(tChange) - 1)]
    
    Fx_2 = [[] for k in range(len(tChange) - 1)]
    Fy_2 = [[] for k in range(len(tChange) - 1)]
    Fz_2 = [[] for k in range(len(tChange) - 1)]
    
    Dead_x = [[] for k in range(len(tChange) - 1)]
    Dead_y = [[] for k in range(len(tChange) - 1)]
    Dead_z = [[] for k in range(len(tChange) - 1)]
        
    Pid_x = [[] for k in range(len(tChange) - 1)]    
    Pid_y = [[] for k in range(len(tChange) - 1)]
    Pid_z = [[] for k in range(len(tChange) - 1)]
    
    i = 0    
    while i < len(tChange) - 1:
        Px_2[i] = Px2[tChange[i]:tChange[i+1]:1]
        Py_2[i] = Py2[tChange[i]:tChange[i+1]:1] 
        Pz_2[i] = Pz2[tChange[i]:tChange[i+1]:1] 
        
        Fx_2[i] = Fx2[tChange[i]:tChange[i+1]:1]
        Fy_2[i] = Fy2[tChange[i]:tChange[i+1]:1] 
        Fz_2[i] = Fz2[tChange[i]:tChange[i+1]:1] 
        
        Pid_x[i] = Pidx[tChange[i]:tChange[i+1]:1]
        Pid_y[i] = Pidy[tChange[i]:tChange[i+1]:1]
        Pid_z[i] = Pidz[tChange[i]:tChange[i+1]:1]

        Dead_x[i] = Deadx[tChange[i]:tChange[i+1]:1]
        Dead_y[i] = Deady[tChange[i]:tChange[i+1]:1] 
        Dead_z[i] = Deadz[tChange[i]:tChange[i+1]:1] 
        
        i += 1
  
#########################Draws graph############################
    
    fig = plt.figure(figsize=(12,21))
    
    def Graph_3D(x_pos, x1, y1, z1, x2, y2, z2, Color1, Color2, Label, x_Label, y_Label, z_Label):
        ax = plt.subplot2grid((17,3), (x_pos,0),colspan = 3, rowspan = 3, projection = '3d')
        ax.plot(x1, y1, z1, color = Color1, label = Label)
        ax.plot(x2 ,y2, z2, color = Color2)    
        ax.set_xlabel(x_Label)
        ax.set_ylabel(y_Label)
        ax.set_zlabel(z_Label)
        ax.autoscale()
        plt.show
        
    def Graph_2D (x_pos, x1, y1, x2, y2, Color1, Color2, x_label, y_label):
        ax = plt.subplot2grid((17,3), (x_pos, 0), colspan = 3, rowspan = 1)
        ax.plot(x1, y1, color = Color1)
        i = 0    
        while i < len(tChange)-1:
            ax.plot(x2[i],y2[i], color = Color2[i])
            i += 1
        ax.set_xlabel(x_label)    
        ax.set_ylabel(y_label)
        ax.autoscale()    
        plt.show
        
#################################################################
    
    #3D    
    ax1 = Graph_3D (0, Px1, Py1, Pz1, Px2, Py2, Pz2, 'black', 'red', 'Position', 'X', 'Y', 'Z')
    ax2 = Graph_3D (4, Fx1, Fy1, Fz1, Fx2, Fy2, Fz2, 'black', 'red', 'Force', 'FX', 'FY', 'FZ')
    
    #2D
    ax3 = Graph_2D (8, t, Px1, t1, Px_2, 'black', Color, '', 'X (Global) [mm]' )
    ax4 = Graph_2D (9, t, Py1, t1, Py_2, 'black', Color, '', 'Y (Global) [mm]')
    ax5 = Graph_2D (10, t, Pz1, t1, Pz_2, 'black', Color, '', 'Z (Global) [mm]')
    ax6 = Graph_2D (11, 0,0 , t1, Pid_x, 'white', Color, '', 'PID X (Tool) [N]')
    ax7 = Graph_2D (12, 0,0 , t1, Pid_y, 'white', Color, '', 'PID y (Tool) [N]')
    ax8 = Graph_2D (13, 0,0 , t1, Pid_z, 'white', Color, '', 'PID z (Tool) [N]')
    ax9 = Graph_2D (14, t, Fx1, t1, Fx_2, 'black', Color, '', 'Fx (Tool) [N]')
    ax10 = Graph_2D (15, t, Fy1, t1, Fy_2, 'black', Color, '', 'Fy (Tool) [N]')
    ax11 = Graph_2D (16, t, Fz1, t1, Fz_2, 'black', Color, 't', 'Fz (Tool) [N]')
    
    subplots_adjust(hspace=0) #gets rid of the distance between the 2d graphs
    
################################################################
          
else:
    print('Unable to find file(s)')  