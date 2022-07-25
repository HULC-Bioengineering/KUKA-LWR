from Packet import Packet
import math

class OpticalTrackerData(Packet):
  """ Subclass of Packet for data captured by an optical tracker.
  Currently holds Dx, Dy, Dz. Modify as needed
  """

  def __init__(self, name="opticaltrackerdata", timestamp=0, dx=0, dy=0, dz=0):
    """ Super class receives timestamp value and default name of 'opticaltrackerdata'

    Args:
      name: packet name. opticaltrackerdata by default.
      timestamp: Timestamp from the Epoch. Default 0 (unset)
      dx: Displacement in X
      dy: Displacement in Y
      dz: Displacement in Z

    Attributes:
      __dx: A float containing displacement in X
      __dy: A float containing displacement in Y
      __dz: A float containing displacement in Z
    """
    Packet.__init__(self, name, timestamp)
    self.__dx = float(dx)
    self.__dy = float(dy)
    self.__dz = float(dz)

  def __str__(self):
    return "Name: %s, Timestamp: %.5f, Dx: %.2f, Dy: %.2f, Dz: %.2f" % (Packet.get_name(self), Packet.get_timestamp(self), self.__dx, self.__dy, self.__dz)
  
  def get_dx(self): return self.__dx
  def get_dy(self): return self.__dy
  def get_dz(self): return self.__dz
  def get_dr(self): return math.sqrt(math.pow(self.__dx,2)+math.pow(self.__dy,2)+math.pow(self.__dz,2))

  def set_dx(self, value): self.__dx = value
  def set_dy(self, value): self.__dy = value
  def set_dz(self, value): self.__dz = value
  
