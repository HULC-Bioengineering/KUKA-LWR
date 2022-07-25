from Sensor import Sensor
from OpticalTrackerData import OpticalTrackerData
import copy

class OpticalTracker(Sensor):
  """ Instantiation of Sensor. Stores a list of OpticalTrackerData. """

  def __init__(self, name="Optical Tracker"):
    """ Initializes "Optical Tracker" as name for every new OpticalTracker Sensor

    Attributes:
      __data: List of OpticalTrackerData
    """
    Sensor.__init__(self,name)
    self.__data = []

  def __str__(self):
    statement = ""
    for i in range(0,len(self.__data)):
      statement += "Element # "+str(i) + "\n"
      statement += str(self.__data[i]) + "\n"
    return statement

  def get_data(self):
    """get a deep copy of the current data list"""
    return copy.deepcopy(self.__data)
  
  def size(self): return len(self.__data)
  
  def addData(self, packet):
    """ Add a new OpticalTrackerData to the list 
    Args:
      packet: OpticalTrackerData to add to the list
    Return:
      Prints message in error
    """
    if isinstance(packet,OpticalTrackerData):
      self.__data.append(packet)
    else:
      print "Add FAIL. Packet NOT OF TYPE OpticalTrackerData"

  def loadDataFromFile(self, filename):
    """ Loads data from filename into OpticalTrackerData objects than stores these objects in the OpticalTracker list. Extremely odd format outputted by LABVIEW
    Args:
      loadcell_file: Input file for loadcell
      optical_tracker_file: Input file for optical tracker
    """
    try:
      reader = open(filename, 'r')
      reader.readline() # pass first line
      count = 0
      for row in reader:
        exploded_row = row.split(" ")
        self.__data.append(OpticalTrackerData(filename+"_"+str(count),exploded_row[0].split('\t')[0],exploded_row[3],exploded_row[7],exploded_row[11]))
        count = count + 1
    except:
      print "FAIL loading optrical tracker data from file"
  
  def accept(self, data_exchange):
    """ Visitor design pattern. Allow visit. """
    data_exchange.visit(opticaltracker=self)
