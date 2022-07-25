from Reamer import Reamer
from Chart import Chart

import argparse
import sys

from gui import Index
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider


############# Command Line Arguments ################

# get command line arguments input, output, loadcel
parser = argparse.ArgumentParser()
parser.add_argument("-lc", "--loadcell", help="file containing load cell data")
parser.add_argument("-ot", "--opticaltracker", help="file containing optical tracker data")
parser.add_argument("-o", "--output", help="output file")
args = parser.parse_args()

# error check command line arguments
if not args.loadcell:
  parser.error("please include loadcell input file")
if not args.opticaltracker:
  parser.error("please include optical tracker input file")
if not args.output:
  parser.error("pleasae include output file")


############ Visitor Pattern Setup #############

# Create objects reamer and chart
r = Reamer()
c = Chart()


# load data from file(s) into reamer
r.loadDataFromFile(args.loadcell, args.opticaltracker)

# setup visitor 
r.accept(c)


############ GUI Setup #########


fig = plt.figure()

ax1 = plt.subplot2grid((5, 5), (0, 0), colspan=5, rowspan=3)
pos_trim = plt.subplot2grid((5, 5), (3, 0), colspan=1, rowspan=1)
pos_normalize = plt.subplot2grid((5, 5), (3, 1), colspan=1, rowspan=1)
pos_bias_lc = plt.subplot2grid((5, 5), (3, 2), colspan=1, rowspan=1)
pos_bias_ot = plt.subplot2grid((5, 5), (3, 3), colspan=1, rowspan=1)
#pos_isolate = plt.subplot2grid((5, 5), (3, 4), colspan=1, rowspan=1)
pos_excel_interp = plt.subplot2grid((5, 5), (4, 0), colspan=1, rowspan=1)
pos_excel_seperated = plt.subplot2grid((5, 5), (4, 1), colspan=1, rowspan=1)
pos_reset = plt.subplot2grid((5, 5), (3, 4), colspan=1, rowspan=1)

plt.tight_layout()

# button names
b_trim = Button(pos_trim, 'Trim',  hovercolor='0.25')
b_normalize = Button(pos_normalize, 'Normalize', hovercolor='0.25')
b_bias_lc = Button(pos_bias_lc, 'Bias LC', hovercolor='0.25')
b_bias_ot = Button(pos_bias_ot, 'Bias OT', hovercolor='0.25')
b_excel_interp = Button(pos_excel_interp, 'Excel', hovercolor='0.25')
b_excel_seperated = Button(pos_excel_seperated, 'Excelv2.0', hovercolor='0.25')
b_reset = Button(pos_reset, 'Reset', hovercolor='0.25')

#sl_isolate = Slider(pos_isolate, 'Isolate', valmin=0, valmax=5, valinit=0, valfmt='%d')


# button on_click
callback = Index(c, ax1, args.output)

b_trim.on_clicked(callback.autoTrim)
b_normalize.on_clicked(callback.normalize)
b_bias_lc.on_clicked(callback.biasLoadCell)
b_bias_ot.on_clicked(callback.biasOpticalTracker)
b_excel_interp.on_clicked(callback.toExcelInterpolated)
b_excel_seperated.on_clicked(callback.toExcelPushSeperated)
b_reset.on_clicked(callback.reset)
#sl_isolate.on_changed(callback.isolate)

############### Running GUI ##############

plt.show()
