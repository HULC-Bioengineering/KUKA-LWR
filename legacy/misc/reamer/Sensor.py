class Sensor(object):
  """ Sensor an interface. Sensor instantiations will vary ex. loadcell, optical, thermal.
  A sensor parses data in a text file and stores data packets of useful time stepped information into a list"""

  def __init__(self, name="Unnamed"):
    """
    Args:
      name: Name of the sensor

    Attributes:
      __name: A string containing the name of the sensor. Defaults to "Unnamed"
      __data: A list containing packet objects for each timestamped measurement
    """
    self.__name = name
    self.__data = []

  def __str__(self):
    statement = ""
    for i in range(0,len(self.__data)):
      statement += "Element # "+str(i) + "\n"
      statement += str(self.__data[i]) + "\n"
    return statement

  def get_name(self): return self.__name
  def set_name(self,name): self.__name = name
  
  def get_data(self): return self.__data
  def set_data(self,data_list): self.__data = data_list
  def addData(self, packet): self.__data.append(packet)
  
  def size(self): return len(self.__data)
