# -*- coding: utf-8 -*-
"""
Description goes here!
"""

import numpy as np
import matplotlib.pyplot as plt
import itertools
import random, glob
import matplotlib.cm as cm, IPython.display as IPdisplay 
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization! 
from PIL import Image as PIL_Image
from images2gif import writeGif

black_grey = itertools.cycle(['#151515', '#696969'])
blues = ['#28ABE3', '#236CF5']
greens = ['#32CB2E', '#008000']
reds = ['#ff0000', '#660000']

# process spatial path and desired forces (dsr path)
# open as/through numpy (because its a csv file) this lets you efficeitnly read
# file into array/storage containers
# should probably be a function
# getting directions, desired spatial path, desired force path then we are
# nomalizing
######################################################################
spatial_path_file = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\paths\\load\\tiny_y.csv';
path = [line.rstrip('\n') for line in open(spatial_path_file)] 
points = [point.split(',') for point in path]
x_dsr = []
y_dsr = []
z_dsr = []
fx_dsr = []
fy_dsr = []
fz_dsr = []
transitions = []
prev_trans = (round(float(points[0][7].strip()) * 1000, 0), round(float(points[0][8].strip()) * 1000, 0), round(float(points[0][9].strip()) * 1000, 0))
dir_latch = 0
directions= []  # 0 == x, 1 == y, 2 == z
for point in points:
    x = round(float(point[7].strip()) * 1000, 0)
    y = round(float(point[8].strip()) * 1000, 0)
    z = round(float(point[9].strip()) * 1000, 0)
    fx_dsr.append(round(float(point[1].strip()), 2))
    fy_dsr.append(round(float(point[2].strip()), 2))
    fz_dsr.append(round(float(point[3].strip()), 2))
    x_dsr.append(x)
    y_dsr.append(y)
    z_dsr.append(z) 

    if not x == prev_trans[0]:
        dir_latch = 0
    if not y == prev_trans[1]:
        dir_latch = 1
    if not z == prev_trans[2]:
        dir_latch = 2
    
    directions.append(dir_latch)
    prev_trans = (x, y, z)     
#directions = directions[1:]  # remove first element as it is meaningless

# normalize
x_dsr_norm = [a - x_dsr[0] for a in x_dsr]
y_dsr_norm = [a - y_dsr[0] for a in y_dsr]
z_dsr_norm = [a - z_dsr[0] for a in z_dsr]
############################################################


# logged file path
# getting PID values, deadbands, measured forces, measured position, time, colors
# then normalize
#######################################################
log_file = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\tiny_y.csv';
lines = [line.rstrip('\n') for line in open(log_file)]  
vals = [x.split(',') for x in lines[2:]]
x_force_msr = []
y_force_msr = []
z_force_msr = []
x_msr = []
y_msr = []
z_msr = []
pid_x = []
pid_y = []
pid_z = []
dead_x = []
dead_y = []
dead_z = []
t = []
t_dsr = [0] 
colors = []  # first element will be removed later
prev_trans = -10000  # initial starting num could also be -inf/inf
t_count = 0;
point_count = 0;
dsr_dir = 0;
for line in vals:
    dead_x.append(round(float(line[0].strip()), 2))
    dead_y.append(round(float(line[1].strip()), 2))
    dead_z.append(round(float(line[2].strip()), 2))
    x_force_msr.append(round(float(line[6].strip()), 2))
    y_force_msr.append(round(float(line[7].strip()), 2))
    z_force_msr.append(round(float(line[8].strip()), 2))
    x_msr.append(round(float(line[15].strip()) * 1000, 2))
    y_msr.append(round(float(line[16].strip()) * 1000, 2))
    z_msr.append(round(float(line[17].strip()) * 1000, 2))
    pid_x.append(round(float(line[18].strip()) * 1000, 2))
    pid_y.append(round(float(line[19].strip()) * 1000, 2))
    pid_z.append(round(float(line[20].strip()) * 1000, 2))
    t.append(t_count)
      
    trans_point = round(float(line[0].strip()) * 1000, 2)
    if not colors:
        if directions[dsr_dir] == 0:
            colors.append(blues[0])
        elif directions[dsr_dir] == 1:
            colors.append(greens[0])
        elif directions[dsr_dir] == 1:
            colors.append(reds[0])
        dsr_dir = dsr_dir + 1
        
    # transition check if its a toggle along the current path or a new path
    elif not trans_point == prev_trans:
        t_dsr.append(t_count)
        # x direction
        if directions[dsr_dir] == 0:
            # toggle check
            if colors[-1] == blues[0]:
                colors.append(blues[1])
            else:
                colors.append(blues[0])
        elif directions[dsr_dir] == 1:
            if colors[-1] == greens[0]:
                colors.append(greens[1])
            else:
                colors.append(greens[0])
        elif directions[dsr_dir] == 2:
            if colors[-1] == reds[0]:
                colors.append(reds[1])
            else:
                colors.append(reds[0])
        dsr_dir = dsr_dir + 1
        
    else:
        colors.append(colors[-1])
        
    point_count = point_count + 1
    prev_trans = trans_point
    t_count += 0.002
t_dsr.append(t_count - 0.002)
    
# normalize
x_msr_norm = [a - x_msr[0] for a in x_msr]
y_msr_norm = [a - y_msr[0] for a in y_msr]
z_msr_norm = [a - z_msr[0] for a in z_msr]

# normalize forces
fx_msr_norm = [a - x_force_msr[0] for a in x_force_msr]
fy_msr_norm = [a - y_force_msr[0] for a in y_force_msr]
fz_msr_norm = [a - z_force_msr[0] for a in z_force_msr]
#####################################################



# all 2D plots should share same X axis but have own Y axis
# should create a function to setup plots in grid
##################################################
colors3d = itertools.cycle(colors)
fig = plt.figure(figsize=(12,21))

# 3d plot
ax1 = plt.subplot2grid((15, 3), (0, 0), colspan=3, rowspan=3, projection='3d')  
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.autoscale()
#ax1.set_ylim(-2,6)
#ax1.set_xlim(-1,4)
#ax1.set_zlim(-0.5, 2.5)

ax2 = plt.subplot2grid((15, 3), (3, 0), colspan=3, rowspan=1)
ax2.set_xlabel('t')
ax2.set_ylabel('X (Global) [mm]')
ax2.autoscale()
#ax2.set_ylim(-1, 4)
#ax2.set_xlim(0, t[-1])

ax3 = plt.subplot2grid((15, 3), (4, 0), colspan=3, rowspan=1)
ax3.set_xlabel('t')
ax3.set_ylabel('Y (Global) [mm]')
ax3.autoscale()
#ax3.set_ylim(-2, 6)
#ax3.set_xlim(0, t[-1])

ax4 = plt.subplot2grid((15, 3), (5, 0), colspan=3, rowspan=1)
ax4.set_xlabel('t')
ax4.set_ylabel('Z (Global) [mm]')
#ax4.set_ylim(-0.5, 2.5)
#ax4.set_xlim(0, t[-1])
ax4.autoscale()

ax5 = plt.subplot2grid((15, 3), (6, 0), colspan=3, rowspan=1)
ax5.set_xlabel('t')
ax5.set_ylabel('PID X (Tool) [mm]')
ax5.autoscale()
#ax5.set_ylim(-0.6, 0.6)
#ax5.set_xlim(0, t[-1])

ax6 = plt.subplot2grid((15, 3), (7, 0), colspan=3, rowspan=1)
ax6.set_xlabel('t')
ax6.set_ylabel('PID Y (Tool) [mm]')
#ax6.set_ylim(-0.6, 0.6)
ax6.autoscale()
#ax6.set_xlim(0, t[-1])

ax7 = plt.subplot2grid((15, 3), (8, 0), colspan=3, rowspan=1)
ax7.set_xlabel('t')
ax7.set_ylabel('PID Z (Tool) [mm]')
#ax7.set_ylim(-0.6, 0.6)
#ax7.set_xlim(0, t[-1])
ax7.autoscale()

ax8 = plt.subplot2grid((15, 3), (9, 0), colspan=3, rowspan=1)
ax8.set_xlabel('t')
ax8.set_ylabel('Fx (Tool) [N]')
ax8.autoscale()
#ax8.set_xlim(0, t[-1])
#ax8.set_ylim(-3,2)

ax9 = plt.subplot2grid((15, 3), (10, 0), colspan=3, rowspan=1)
ax9.set_xlabel('t')
ax9.set_ylabel('Fy (Tool) [N]')
ax9.autoscale()
#ax9.set_xlim(0, t[-1])
#ax9.set_ylim(4.5,7.5)

ax10 = plt.subplot2grid((15, 3), (11, 0), colspan=3, rowspan=1)
ax10.set_xlabel('t')
ax10.set_ylabel('Fz (Tool) [N]')
#ax10.set_xlim(0, t[-1])
#ax10.set_ylim(-22,-18.5)
ax10.autoscale()

# 3d plot
ax11 = plt.subplot2grid((15, 3), (12, 0), colspan=3, rowspan=3, projection='3d')  
ax11.set_xlabel('FX')
ax11.set_ylabel('FY')
ax11.set_zlabel('FZ')
#ax11.set_xlim(-3,2)
#ax11.set_ylim(4.5, 7.5)
#ax11.set_zlim(-22,-18.5)
ax11.autoscale()
##########################################


# line segments will improve efficiency here
#######################################
prev_color = "#FFFFFF"
extent = ax11.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
forces_3d = "Forces3D_"
counter = 0
ps = [(ts,x,y,z, pi_x, pi_y, pi_z, fx, fy, fz, db_x, db_y, db_z, fx_norm, fy_norm, fz_norm) for (ts,x,y,z, pi_x, pi_y, pi_z, fx, fy, fz, db_x, db_y, db_z, fx_norm, fy_norm, fz_norm) in zip(t,x_msr_norm, y_msr_norm, z_msr_norm, pid_x, pid_y, pid_z, x_force_msr, y_force_msr, z_force_msr, dead_x, dead_y, dead_z, fx_msr_norm, fy_msr_norm, fz_msr_norm)]
for start, end in zip(ps[:-1], ps[1:]):
    ts, x, y, z, pi_x, pi_y, pi_z, fx, fy, fz, db_x, db_y, db_z, fx_norm, fy_norm, fz_norm = zip(start, end)
    c = next(colors3d)
    cc = next(black_grey)
    ax1.plot(x, y, z, color=c)  
    ax2.plot(ts, x, color=c)
    ax3.plot(ts, y, color=c)
    ax4.plot(ts, z, color=c)
    ax5.plot(ts, pi_x, color=c)
    ax6.plot(ts, pi_y, color=c)
    ax7.plot(ts, pi_z, color=c)
    ax8.plot(ts, fx, color=c)
    ax8.plot(ts, db_x, color=cc)
    ax9.plot(ts, fy, color=c)
    ax9.plot(ts, db_y, color=cc)
    ax10.plot(ts, fz, color=c)
    ax10.plot(ts, db_z, color=cc)
    ax11.plot(fx, fy, fz, color=c)
    ax11.plot(db_x, db_y, db_z, color=cc)
    '''
    if c is not prev_color:
        fig.savefig('C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\images\\' + forces_3d + str(counter) + '.png', dpi=50)
        counter = counter + 1
        prev_color = c
    ''' 
ps = [(ts,x,y,z,fx,fy,fz) for (ts,x,y,z,fx,fy,fz) in zip(t_dsr,x_dsr_norm, y_dsr_norm, z_dsr_norm, fx_dsr, fy_dsr, fz_dsr)]
for start, end in zip(ps[:-1], ps[1:]):
    ts, x, y, z, fx, fy, fz = zip(start, end)
    c = next(black_grey)
    ax1.plot(x, y, z, color=c)  
    ax2.plot(ts, x, color=c)
    ax3.plot(ts, y, color=c)
    ax4.plot(ts, z, color=c)
    ax8.plot(ts, fx, color=c)
    ax9.plot(ts, fy, color=c)
    ax10.plot(ts, fz, color=c)
####################################################
    
    
# line cross section
#d = abs(cross(Q2-Q1,P-Q1))/abs(Q2-Q1);
#fig.savefig('C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\logged\\test.png', dpi=600)
plt.show()
#plt.close()

# ...instead, create an animated gif of all the frames, then display it inline 
#images = [PIL_Image.open(image) for image in glob.glob('C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\images\\*.png')]
#file_path_name = 'C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\images\\' + '3D' + '.gif'
#writeGif(file_path_name, images, duration=0.2)
#IPdisplay.Image(url=file_path_name)