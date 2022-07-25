from OpticalTracker import OpticalTracker
from LoadCell import LoadCell
from DataExchange import DataExchange

class Reamer(object):
  """ A Reamer is a collection of sensors. """

  def __init__(self, opticaltracker=OpticalTracker(), loadcell=LoadCell()):
    """
    Args:
      opticaltracker: An optical tracker. Will create empty optical tracker by default
      loadcell: A load cell. Will create empty load cell by default

    Attributes:
      __ot: A list of optical trackers
      __lc: A list of load cells
    """
    if not isinstance(opticaltracker, OpticalTracker) or not isinstance(loadcell, LoadCell):
      print "Improper initialization of Reamer. Should be types OpticalTracker and LoadCell"
    else:
      self.__ot = opticaltracker
      self.__lc = loadcell

  def __str__(self):
    return str(self.__ot) + str(self.__lc)

  def loadDataFromFile(self, loadcell_file, optical_tracker_file):
    """ Loads data from input file into loadcell and opticaltracker lists respectfully. Requires one load cell and one optical tracker file
    Args:
      loadcell_file: Input file for loadcell
      optical_tracker_file: Input file for optical tracker
    """
    self.__ot.loadDataFromFile(optical_tracker_file)
    self.__lc.loadDataFromFile(loadcell_file)
    
  def accept(self, data_exchange):
    """ Visitor design pattern. Allows for a DataExchange object to visit to extract information
    Args:
      data_exchange: DataExchange object. Called via: Reamer.visit(DataExchange Obj)
    """
    data_exchange.visit(loadcell=self.__lc, opticaltracker=self.__ot)
