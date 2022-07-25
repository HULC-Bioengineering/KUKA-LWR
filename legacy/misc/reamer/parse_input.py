import sys
import math

from Sensor import Sensor
from Loadcell import Loadcell
class LoadCell(Sensor):

  def __init__(self,timestamp,fx,fy,fz):
    Sensor.__init__(self,"LoadCell")
    self.__timestamp = float(timestamp)
    self.__fx = float(fx)
    self.__fy = float(fy)
    self.__fz = float(fz)

  def __str__(self):
    return str(self.__timestamp) + " " + str(self.__fx) + " " + str(self.__fy) + " " + str(self.__fz) + "\n"

  def get_timestamp(self): return self.__timestamp
  def get_fx(self): return self.__fx
  def get_fy(self): return self.__fy
  def get_fz(self): return self.__fz

  def set_timestamp(self, value): self.__timestamp = value
  def set_fx(self, value): self.__fx = value
  def set_fy(self, value): self.__fy = value
  def set_fz(self, value): self.__fz = value

  def write(self):
    # return str(self.__timestamp) + " " + str(self.__fx) + " " + str(self.__fy) + " " + str(self.__fz) + "\n"
    return str(self.__timestamp) + " " + str(math.sqrt(self.__fx*self.__fx + self.__fy*self.__fy + self.__fz*self.__fz)) + "\n"


class OpticalTracker(Sensor):
  
  def __init__(self, timestamp=0, dx=0, dy=0, dz=0):
    Sensor.__init__(self,"OpticalTracker")
    self.__timestamp = float(timestamp)
    self.__dx = float(dx)
    self.__dy = float(dy)
    self.__dz = float(dz)

  def __str__(self):
    return self.__timestamp + " " + self.__dx + " " + self.__dy + " " + self.__dz + "\n"
  
  def get_timestamp(self): return self.__timestamp
  def get_dx(self): return self.__dx
  def get_dy(self): return self.__dy
  def get_dz(self): return self.__dz

  def set_timestamp(self, value): self.__timestamp = value
  def set_dx(self, value): self.__dx = value
  def set_dy(self, value): self.__dy = value
  def set_dz(self, value): self.__dz = value
  
  def write(self):
    #return str(self.__timestamp) + " "+ " "+ " "+ " " + str(self.__dx) + " " + str(self.__dy) + " " + str(self.__dz) + "\n"
    return str(self.__timestamp) + " " + " " + str(math.sqrt(self.__dx*self.__dx + self.__dy*self.__dy + self.__dz*self.__dz)) + "\n"

class Reamer(object):
  _LOADCELL_FILE = './loadcell_input'
  _OPTICAL_FILE = './optical_input'

  __loadcell = []
  __optical_tracker = []
  __output = []

  def __init__(self):
    self.__createOutput()

  def __createOutput(self):
    self.__read_loadcell()
    self.__read_optical_tracker()
  
    i = 0
    j = 0

    while i < len(self.__loadcell) and j < len(self.__optical_tracker):
      if self.__loadcell[i].get_timestamp() < self.__optical_tracker[j].get_timestamp():
        self.__output.append(self.__loadcell[i])
        i = i + 1
      else:
        self.__output.append(self.__optical_tracker[j])
        j = j + 1

    # ending conditions check if still remaining in i or j
    while i < len(self.__loadcell):
      self.__output.append(self.__loadcell[i])
      i = i + 1

    while j < len(self.__optical_tracker):
      self.__output.append(self.__optical_tracker[j])
      j = j + 1

    print len(self.__output)
    print len(self.__loadcell)
    print len(self.__optical_tracker)

  def __read_loadcell(self):
    reader = open(self._LOADCELL_FILE, 'r')
    for row in reader:
      exploded_row = row.split(",")
      self.__loadcell.append(LoadCell(exploded_row[0],exploded_row[1],exploded_row[2],exploded_row[3]))

  def __read_optical_tracker(self):
    reader = open(self._OPTICAL_FILE, 'r')
    reader.readline() # pass first line
    for row in reader:
      exploded_row = row.split(" ")
      self.__optical_tracker.append(OpticalTracker(exploded_row[0].split('\t')[0],exploded_row[3],exploded_row[7],exploded_row[11]))
  def get_loadcell(self): return self.__loadcell
  def get_optical_tracker(self): return self.__optical_tracker
  def get_output(self): return self.__output

  def size_loadcell(): return len(self.__loadcell)
  def size_optical_tracker(): return len(self.__optical_tracker)
  def size_output() : return len(self.__output)

  def ranges(self,start, end):
    if start > end:
      return []

    start_value = self.__output[0].get_timestamp()
    end_value = self.__output[-1].get_timestamp()

    start_counter = 0
    end_counter = 0

    for points in self.__output:
      if start_value+start > points.get_timestamp():
        start_counter = start_counter + 1
      if end_value-end < points.get_timestamp():
        end_counter = end_counter + 1

    return self.__output[start_counter:end_counter]

  def duration(self):
    return self.__output[-1].get_timestamp() - self.__output[0].get_timestamp()

  def trim_start(self,seconds):
    start = self.__output[0].get_timestamp()

    position = 0
    for points in self.__output:
      if start+seconds < points.get_timestamp():
        break
      else:
        position = position + 1

    self.__output = self.__output[position:]

  def trim_end(self,seconds):
    end = self.__output[-1].get_timestamp()

    position = 0
    for points in self.__output:
      if end-seconds < points.get_timestamp():
        break
      else:
        position = position + 1

    self.__output = self.__output[0:position]

  def recreateOutput(self):
    self.__createOutput()

  def normalizeTimestamp(self):
    start = self.__output[0].get_timestamp()

    pos = 0
    for points in self.__output:
      points.set_timestamp(points.get_timestamp() - start)
      pos = pos + 1

  def normalizeFx(self):
      start_loc = 0
      while not isinstance(self.__output[start_loc],LoadCell):
        start_loc = start_loc + 1
      
      start = self.__output[start_loc].get_fx()

      pos = 0
      for points in self.__output:
        if isinstance(points,LoadCell):
          points.set_fx(points.get_fx() - start)
        pos = pos + 1
  
  def normalizeFy(self):
      start_loc = 0
      while not isinstance(self.__output[start_loc],LoadCell):
        start_loc = start_loc + 1
      
      start = self.__output[start_loc].get_fy()

      pos = 0
      for points in self.__output:
        if isinstance(points,LoadCell):
          points.set_fy(points.get_fy() - start)
        pos = pos + 1
  
  def normalizeFz(self):
      start_loc = 0
      while not isinstance(self.__output[start_loc],LoadCell):
        start_loc = start_loc + 1
      
      start = self.__output[start_loc].get_fz()

      pos = 0
      for points in self.__output:
        if isinstance(points,LoadCell):
          points.set_fz(points.get_fz() - start)
        pos = pos + 1
  
  def normalizeDx(self):
      start_loc = 0
      while not isinstance(self.__output[start_loc],OpticalTracker):
        start_loc = start_loc + 1
      
      start = self.__output[start_loc].get_dx()

      pos = 0
      for points in self.__output:
        if isinstance(points,OpticalTracker):
          points.set_dx(points.get_dx() - start)
        pos = pos + 1
  
  def normalizeDy(self):
      start_loc = 0
      while not isinstance(self.__output[start_loc],OpticalTracker):
        start_loc = start_loc + 1
      
      start = self.__output[start_loc].get_dy()

      pos = 0
      for points in self.__output:
        if isinstance(points,OpticalTracker):
          points.set_dy(points.get_dy() - start)
        pos = pos + 1
  
  def normalizeDz(self):
      start_loc = 0
      while not isinstance(self.__output[start_loc],OpticalTracker):
        start_loc = start_loc + 1
      
      start = self.__output[start_loc].get_dz()

      pos = 0
      for points in self.__output:
        if isinstance(points,OpticalTracker):
          points.set_dz(points.get_dz() - start)
        pos = pos + 1
  
  ### Visitor Design Pattern ###
  ### + Observer Design pattern ###
def main():
  r = Reamer()
  r.trim_start(1)
  r.trim_end(1)
  r.normalizeTimestamp()
  r.normalizeFx()
  r.normalizeFy()
  r.normalizeFz()
  r.normalizeDx()
  r.normalizeDy()
  r.normalizeDz()

  with open('output2.csv', 'wb') as f:
    count = 0
    for items in r.ranges(0,100):
      f.write(items.write())

if __name__ == '__main__':
  main()
