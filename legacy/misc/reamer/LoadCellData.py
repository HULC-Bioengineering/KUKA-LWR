from Packet import Packet
import math

class LoadCellData(Packet):
  """ Subclass of Packet for data captured by a load cell.
  Currently holds Fx, Fy, Fz. Modify as needed
  """

  def __init__(self,name="loadcelldata",timestamp=0,fx=0,fy=0,fz=0):
    """ Super class receives timestamp value

    Args:
      name: Name of the entry. Default is "loadcelldata
      timestamp: Timestamp from the Epoch. Default 0 (unset)
      fx: Force in X
      fy: Force in Y
      fz: Force in Z

    Attributes:
      __fx: A float containing force in X
      __fy: A float containing force in Y
      __fz: A float containing force in Z
    """
    Packet.__init__(self, name, timestamp)
    self.__fx = float(fx)
    self.__fy = float(fy)
    self.__fz = float(fz)

  def __str__(self):
    return "Name: %s, Timestamp: %.5f, Fx: %.2f, Fy: %.2f, Fz: %.2f" % (Packet.get_name(self), Packet.get_timestamp(self), self.__fx, self.__fy, self.__fz)

  def get_fx(self): return self.__fx
  def get_fy(self): return self.__fy
  def get_fz(self): return self.__fz
  def get_fr(self): return math.sqrt(math.pow(self.__fx,2)+math.pow(self.__fy,2)+math.pow(self.__fz,2))

  def set_fx(self, value): self.__fx = value
  def set_fy(self, value): self.__fy = value
  def set_fz(self, value): self.__fz = value
