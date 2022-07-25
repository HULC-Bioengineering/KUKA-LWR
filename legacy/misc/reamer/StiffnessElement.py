
class StiffnessElement(object):

  def __init__(self, timestamp=0, fr=0, dr=0):
    self.__timestamp = timestamp
    self.__fr = fr
    self.__dr = dr

  def __str__(self):
    return "Timestamp: %s, Fr: %.2f, Dr: %.2f" % (self.__timestamp, self.__fr, self.__dr)

  def set_fr(self, value):
    self.__fr = value

  def set_dr(self, value):
    self.__dr = value

  def set_timestamp(self, value):
    self.__timestamp = value

  def get_timestamp(self): return self.__timestamp
  def get_fr(self): return self.__fr
  def get_dr(self): return self.__dr

  def get_stiffness(self):
    if self.__dr!=0:
      return (self.__fr/self.__dr) * 1000 # N/m
    else:
      return 0
