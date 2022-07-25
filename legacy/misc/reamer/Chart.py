import sys
import math
import os
import copy
from LoadCell import LoadCell
from DataExchange import DataExchange
from LoadCellData import LoadCellData
from OpticalTrackerData import OpticalTrackerData
from OpticalTracker import OpticalTracker
from StiffnessElement import StiffnessElement

#from pylab import *
#import matplotlib.pyplot as plt

class Chart(DataExchange):
  def __init__(self):
    self.__lc = [] # list of lists
    self.__ot = [] # list of lists
    self.__x = []
    self.__k = []

  def visit(self, loadcell=None, opticaltracker=None):
    if loadcell != None:
      self.addLoadCell(loadcell.get_data())
    if opticaltracker != None:
      self.addOpticalTracker(opticaltracker.get_data())

  def addLoadCell(self, loadcell):
    self.__lc.append(copy.deepcopy(loadcell))
    for i in range(0,len(loadcell)):
      self.__x.append(copy.deepcopy(loadcell[i]))
    self.__x = sorted(self.__x, key=lambda sensor: sensor.get_timestamp())
  
  def addOpticalTracker(self, ot):
    self.__ot.append(copy.deepcopy(ot))
    for i in range(0,len(ot)):
      self.__x.append(copy.deepcopy(ot[i]))
    self.__x = sorted(self.__x, key=lambda sensor: sensor.get_timestamp())

  def normalize(self):
    start = float(self.__x[0].get_timestamp())
    for i in range(0,len(self.__x)):
      self.__x[i].set_timestamp(float(self.__x[i].get_timestamp()) - start)
  
  def printTimestamp(self):
    for item in self.__x:
      print item.get_timestamp()
  
  def reset(self):
    self.__x = []
   
    # add from _ot and _lc
    for loadcell in self.__lc:
      for i in range(0,len(loadcell)):
        self.__x.append(copy.deepcopy(loadcell[i]))
    for opticaltracker in self.__ot:
      for i in range(0,len(opticaltracker)):
        self.__x.append(copy.deepcopy(opticaltracker[i]))

    self.__x = sorted(self.__x, key=lambda sensor: sensor.get_timestamp())
    
  def printLC(self):
    for item in self.__x:
      if isinstance(item,LoadCellData):
        print item.get_fx()
        print item.get_fy()
        print item.get_fz()

  def printOT(self):
    for item in self.__x:
      if isinstance(item, OpticalTrackerData):
        print item.get_dr()
        


  def selectRange(self, start, end):
    """ start and end in seconds """
    if start > end:
      print "ERROR: Select Range - Starting value is greater than ending value"
      return
    start_value = float(self.__x[0].get_timestamp())
    end_value = float(self.__x[-1].get_timestamp())

    start_counter = 0
    end_counter = 0
    
    for packet in self.__x:
      if packet.get_timestamp() <= start + start_value:
        start_counter = start_counter + 1
      if packet.get_timestamp() <= end + start_value:
        end_counter = end_counter + 1

    self.__x = self.__x[start_counter:end_counter]

  def duration(self):
    return self.__x[-1].get_timestamp() - self.__x[0].get_timestamp()

  def trimStart(self, seconds):
    start = float(self.__x[0].get_timestamp())

    position = 0
    for packets in self.__x:
      if start + seconds < packets.get_timestamp():
        break
      else:
        position = position + 1

    self.__x = self.__x[position:]
  
  def trimEnd(self, seconds):
    end = float(self.__x[-1].get_timestamp())

    position = 0
    for packets in self.__x:
      if end - seconds < packets.get_timestamp():
        break
      else:
        position = position + 1

    self.__x = self.__x[0:position]
  
  def autoTrim(self):
    
    # start point
    start = 1
    if isinstance(self.__x[0], LoadCellData):
      while isinstance(self.__x[start], LoadCellData):
        start = start + 1
    elif isinstance(self.__x[0], OpticalTrackerData):
      while isinstance(self.__x[start], OpticalTrackerData):
        start = start + 1

    end = len(self.__x) - 1
    if isinstance(self.__x[end], LoadCellData):
      while isinstance(self.__x[end], LoadCellData):
        end = end - 1
    elif isinstance(self.__x[end], OpticalTrackerData):
      while isinstance(self.__x[end], OpticalTrackerData):
        end = end -1

    self.__x = self.__x[start-1:end+2]

  def biasLoadCell(self):
    start_counter = 0
    while not isinstance(self.__x[start_counter], LoadCellData):
      start_counter = start_counter + 1

    start_x = float(self.__x[start_counter].get_fx())
    start_y = float(self.__x[start_counter].get_fy())
    start_z = float(self.__x[start_counter].get_fz())
  
    for packets in self.__x:
      if isinstance(packets, LoadCellData):
        packets.set_fx(packets.get_fx() - start_x)
        packets.set_fy(packets.get_fy() - start_y)
        packets.set_fz(packets.get_fz() - start_z)
    
  def biasOpticalTracker(self):
    start_counter = 0
    while not isinstance(self.__x[start_counter], OpticalTrackerData):
      start_counter = start_counter + 1

    start_x = float(self.__x[start_counter].get_dx())
    start_y = float(self.__x[start_counter].get_dy())
    start_z = float(self.__x[start_counter].get_dz())
  
    for packets in self.__x:
      if isinstance(packets, OpticalTrackerData):
        packets.set_dx(packets.get_dx() - start_x)
        packets.set_dy(packets.get_dy() - start_y)
        packets.set_dz(packets.get_dz() - start_z)
  
  def printX(self):
    for item in self.__x:
      print item

  def printOpticalTrackingData(self):
    for sensor in self.__ot:
      for packets in sensor:
        print packets

  def toExcel(self, filename):
    try:

      if not os.path.exists(os.path.abspath(os.path.join(filename, os.pardir))):
        os.makedirs(os.path.abspath(os.path.join(filename, os.pardir)))
      
      f = open(filename, 'w')
      f.write("Timestamp,Resultant_Force,Resultant_Displacement\n")
      for packet in self.__x:
        line = str(packet.get_timestamp()) + "," 
        if isinstance(packet, LoadCellData):
          line += str(packet.get_fr())
        elif isinstance(packet, OpticalTrackerData):
          line += "," +str(packet.get_dr())
        f.write(line+"\n")
    except IOError as e:
      print "Error opening output file for writing...impressive"

  def calculateStiffness(self):
    self.__interpolate()
  
  def isolate1push(self, threshold):
    if self.__k == []:
      self.__interpolate()
    
    new_k = []

    for element in self.__k:
      if element.get_fr() > threshold:
        new_k.append(element)

    self.__k = new_k

  def toExcelInterpolated(self, filename):
    try:

      if not os.path.exists(os.path.abspath(os.path.join(filename, os.pardir))):
        os.makedirs(os.path.abspath(os.path.join(filename, os.pardir)))
      
      f = open(filename, 'w')
      f.write("Timestamp,Resultant_Force,Resultant_Displacement,Stiffness\n")
      if self.__k == []:
        self.__interpolate()

      for element in self.__k:
        line = str(element.get_timestamp()) + "," 
        line += str(element.get_fr()) + ","
        line += str(element.get_dr()) + ","
        line += str(element.get_stiffness())
        f.write(line+"\n")

      print "toExcelInterpolated successful"

    except IOError as e:
      print "Error opening output file for writing...impressive"

  def toExcelPushSeperated(self, filename):
    try:
      if not os.path.exists(os.path.abspath(os.path.join(filename, os.pardir))):
        os.makedirs(os.path.abspath(os.path.join(filename, os.pardir)))

      f = open(filename, 'w')
      f.write("Timestamp,Resultant_Force,Resultant_Displacement,Stiffness\n")
      if self.__k == []:
        self.__interpolate()

      prev = self.__k[0].get_timestamp()
      for element in self.__k:
        line = str(element.get_timestamp()) + "," 
        line += str(element.get_fr()) + ","
        line += str(element.get_dr()) + ","
        line += str(element.get_stiffness())
        if element.get_timestamp() - prev > .25:
          for i in range(0,5): f.write("\n")
        f.write(line+"\n")
        prev = element.get_timestamp()

      print "toExcelPushSeperated successful"

    except IOError as e:
      print "Error opening output file for writing...impressive"


  def dumpTimeLC(self):
    x = []
    for item in self.__x:
      if isinstance(item,LoadCellData):
        x.append(item.get_timestamp())
    return x
  
  def dumpTimeOT(self):
    x = []
    for item in self.__x:
      if isinstance(item,OpticalTrackerData):
        x.append(item.get_timestamp())
    return x

  def dumpLC(self):
    x = []
    for item in self.__x:
      if isinstance(item,LoadCellData):
        x.append(item.get_fr())
    return x

  def dumpOT(self):
    x = []
    for item in self.__x:
      if isinstance(item,OpticalTrackerData):
        x.append(item.get_dr())
    return x

  def isolate(self, start, end):
    self.__x = self.__x[start:end]

  ''' 2 pass: 1) loadcell interp 2) optical tracker interp
      returns a list of turpals'''
  def __interpolate(self):

    stiff_list = []

    # pass 1: loadcell interoplation
    interp1 = False # bool
    interp1_val = 0
    interp2 = False
    interp2_val = 0

    for loc in range(0,len(self.__x)):

      # if found a load cell value
      if isinstance(self.__x[loc], LoadCellData):
        if interp2 == True:
          interp2_val = loc
          self.__interpolateLC(interp1_val, interp2_val, stiff_list)
          interp2 = False

        if interp2 == False:
          interp1 = True
          interp1_val = loc
          stiff_list.append(StiffnessElement(timestamp=self.__x[loc].get_timestamp(), fr=self.__x[loc].get_fr())) # add lc data to stiffness element        
      else: # optical tracker element
        interp2 = True


    # pass 2: optical tracker interoplation
    interp1 = False
    interp1_val = 0
    interp2 = False
    interp2_val = 0

    for loc in range(0, len(self.__x)):

      # if found an optical tracker value
      if isinstance(self.__x[loc], OpticalTrackerData):
        if interp2 == True:
          interp2_val = loc
          self.__interpolateOT(interp1_val, interp2_val, stiff_list)
          interp2 = False

        if interp2 == False:
          interp1 = True
          interp1_val = loc
          try:
            stiff_list[loc].set_dr(self.__x[loc].get_dr()) # add lc data to stiffness element        
          except IndexError:
            pass

      else: # optical tracker element
        interp2 = True
    
    self.__k = stiff_list

  ''' interpolate values using timestamp '''
  def __interpolateLC(self, start, end, stiff_list):
    for element in range(start+1, end):

      # y = y_a + (y_b - y_a)[(x-x_a)/(x_b-x_a)]
      interp_fr = self.__x[start].get_fr() + ((self.__x[end].get_fr() - self.__x[start].get_fr()) * (( self.__x[element].get_timestamp() - self.__x[start].get_timestamp()) / (self.__x[end].get_timestamp() - self.__x[start].get_timestamp())))
      #print "Element: " + str(element) + " Value: " + str(interp_fr)

      stiff_list.append(StiffnessElement(timestamp=self.__x[element].get_timestamp(), fr=interp_fr))

  ''' interpolate values using timestamp '''
  def __interpolateOT(self, start, end, stiff_list):
    for element in range(start+1, end):

      # y = y_a + (y_b - y_a)[(x-x_a)/(x_b-x_a)]
      interp_dr = self.__x[start].get_dr() + ((self.__x[end].get_dr() - self.__x[start].get_dr()) * (( self.__x[element].get_timestamp() - self.__x[start].get_timestamp()) / (self.__x[end].get_timestamp() - self.__x[start].get_timestamp())))
      #print "Element OT: " + str(element) + " Value: " + str(interp_dr)
      try: 
        stiff_list[element].set_dr(interp_dr)
      except IndexError:
        pass # last element value is undetermined
  
