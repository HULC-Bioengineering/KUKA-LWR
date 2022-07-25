from Sensor import Sensor
from LoadCellData import LoadCellData
import copy
class LoadCell(Sensor):
  """A list containing all LoadCellData related to a specific loadcell"""
  
  def __init__(self, name="loadcell"):
    Sensor.__init__(self, name)
    self.__data = []
  
  def __str__(self):
    statement = ""
    for i in range(0,len(self.__data)):
      statement += "Element # "+str(i) + "\n"
      statement += str(self.__data[i]) + "\n"
    return statement

  def get_data(self):
    """ Returns a deep copy of the current data"""
    x = copy.deepcopy(self.__data)
    return x
  
  def size(self): return len(self.__data)

  def addData(self, packet):
    """ Add packet to load cell. Prints an error message if packet is not of 
    type LoadCellData.
    """
    if isinstance(packet,LoadCellData):
      self.__data.append(packet)
    else:
      print "Add FAIL. NOT LOADCELLDATA packet"

  def loadDataFromFile(self, filename):
    """ Load data from an input file filename. Data is expected to be in rows
    in the format: timestamp,fx,fy,fx """
    try:
      reader = open(filename, 'r')
      count = 0
      for row in reader:
        exploded_row = row.split(",")
        self.__data.append(LoadCellData(filename+"_"+str(count),exploded_row[0],exploded_row[1],exploded_row[2],exploded_row[3]))
        count = count + 1
    except:
      print "FAIL loading load cell data from file"
  
  def accept(self, data_exchange):
    """ Visitor design pattern. Allow for a visit. """
    data_exchange.visit(loadcell=self)

