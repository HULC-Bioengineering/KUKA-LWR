import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

class Index:

  # takes a chart object and preforms necessary actions
  def __init__(self, chart, ax, output_file):
    self.__chart = chart
    self.__ax = ax
    self.__output_file = output_file
    self.__plot_init()

    # data for line values
    x = self.__chart.dumpTimeLC()
    y = self.__chart.dumpLC()

    x2 = self.__chart.dumpTimeOT()
    y2 = self.__chart.dumpOT()

    # plotting data
    self.__l, = self.__ax.plot(x, y, lw=2)
    self.__l2, = self.__ax.plot(x2, y2, lw=2)

    self.__isolated = 0
  
  def __reload(self):

    self.__l.set_xdata(self.__chart.dumpTimeLC())
    self.__l.set_ydata(self.__chart.dumpLC())

    self.__l2.set_xdata(self.__chart.dumpTimeOT())
    self.__l2.set_ydata(self.__chart.dumpOT())

    self.__ax.relim()
    self.__ax.autoscale()

    plt.draw()

  def autoTrim(self, event):
    print "Autotrim"
    self.__chart.autoTrim()
    self.__reload()

  def normalize(self, event):
    print "Normalize"
    self.__chart.normalize()
    self.__reload()

  def biasLoadCell(self, event):
    print "Bias Load Cell"
    self.__chart.biasLoadCell()
    self.__reload()

  def biasOpticalTracker(self, event):
    print "Bias Optical Tracker"
    self.__chart.biasOpticalTracker()
    self.__reload()

  def isolate(self, event):
    print "Isolate"
    self.__chart.isolate1push(4)
    self.__reload()

  def toExcelInterpolated(self, event):
    print "to Excel Interpolated"
    self.__chart.toExcelInterpolated(self.__output_file)
    self.__reload()

  def toExcelPushSeperated(self, event):
    print "to Excel Push Seperated"
    self.__chart.toExcelPushSeperated(self.__output_file)
    self.__reload()

  def reset(self, event):
    print "Reset"
    self.__chart.reset()
    self.__reload()

  def __plot_init(self, fontsize=12):
    self.__ax.plot([1])
    self.__ax.locator_params(nbins=3)
    self.__ax.set_xlabel('Time', fontsize=fontsize)
    self.__ax.set_ylabel('Force or Displacement', fontsize=fontsize)
    self.__ax.set_title('Reamer Results', fontsize=fontsize)

  def isolate(self, val):
    print int(val)
    if int(val) != self.__isolated:
      self.__chart.isolate1push(val)
      print "change"
      self.__isolated = int(val)

    self.__reload()
    
