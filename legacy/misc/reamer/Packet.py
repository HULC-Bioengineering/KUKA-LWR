class Packet(object):
  """ Packet is an abstract class for information packets from various devices"""

  def __init__(self, name="Unnamed", timestamp=0):
    """
    Args:
      timestamp: Timestamp from the Epoch. Default 0 (unset)
      name: Name for this packet. Default 'Unnamed'

    Attributes:
      __name: A string containg the name of the packet
      __timestamp: A float containing the timestamp in s from the Epoch
    """
    self.__name = name
    self.__timestamp = float(timestamp)

  def __str__(self):
    return "Name: %s, Timestamp: %.5f" % (self.__name, self.__timestamp)

  def get_name(self): return self.__name
  def set_name(self, name): self.__name = name

  def get_timestamp(self): return self.__timestamp
  def set_timestamp(self, value): self.__timestamp = value
