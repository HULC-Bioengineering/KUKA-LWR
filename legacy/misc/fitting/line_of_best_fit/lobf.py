import sys
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import numpy as np


class Point(object):
  """ Organizes point data. """

  def __init__(self, x, y, z):
    """ Initialises "x", "y" and "z" for every point
    Args:
      x: the x value from a point
      y: the y value from a point
      z: the z value from a point
    
    Attributes:
      __x: a value for x
      __y: a value for y
      __z: a value for z
    """

    self.__x = float(x)
    self.__y = float(y)
    self.__z = float(z)

  def __str__(self):
    return "X: %.3f, Y: %.3f, Z: %.3f" % (self.__x, self.__y, self.__z)
  
  def get_x(self): return self.__x
  def get_y(self): return self.__y
  def get_z(self): return self.__z

class Lobf(object):
  """ Line of best fit for all points. Input file model example:
    X Y Z
    0 1 2
    56.2 41.1 64
    """

  def __init__(self, filename):
    """ Calcualtes line of best fit in x, y, z, and 3D
    Args:
     filename: Name of the input file that is being read 
    Attributes:
      __points: List of all the points
      __n: Amount of points in the points list
      __populatePoints: Fills in the __points list with points from the file
    """
    self.__points = [] # Sets the list where all points will be stored
    self.__n = 0
    self.__populatePoints(filename)
    self.__xy = self.__fit2D(x=self.__getX(),y=self.__getY())
    self.__yz = self.__fit2D(y=self.__getY(),z=self.__getZ())
    self.__zx = self.__fit2D(z=self.__getZ(),x=self.__getX())
    
    print self.__xy
    print self.__yz
    print self.__zx

    ###########
    start_y = self.__xy[0]
    start_x = 0
    end_y = self.__xy[0] + self.__xy[1]*80
    end_x = 80
    ###########

    ######
    start_z = self.__yz[0]
    end_z = self.__yz[0] + self.__yz[1]*end_y
    ######


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(self.__getX(), self.__getY(), self.__getZ(), c='r', marker='x')
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    print "END"
    print end_x
    print end_y
    print end_z
    print "START"
    print start_x
    print start_y
    print start_z
    ax.plot([start_x, end_x], [start_y, end_y])
    ax.plot([0,0],[start_y, end_y],[start_z, end_z])
    ax.plot([start_x,end_x],[0,0],[start_z, end_z])
    ax.plot([start_x,end_x],[start_y, end_y],[start_z, end_z])
    plt.show()

  def __str__(self):
    output = ""
    for point in self.__points:
      output = output + str(point) + "\n"
    return output.strip("\n")

  def __populatePoints(self, filename): 
    """ Will read the points from the file and place them into the list, points
    Args:
      filename: name of read file with all the points
    Return:
      Adds point coordinates to the points list
      Prints message in error
    """
    try:
      f = open(filename, 'r')
      f.readline() # pass first line
      
      n = 0
      for row in f:
        row = row.strip("\n")
        exploded_row = row.split(" ")
        p = Point(exploded_row[0], exploded_row[1], exploded_row[2])
        self.__points.append(p)
        n = n + 1
    
    except IOError as e:
      print e.strerror + " failed to open"

    self.__n = n

  def __fit2D(self,x=None,y=None,z=None):
    """ Recieves 2 coordinates and sends the variables to "get_a" as dependent and independent to calculate the 2D line of best fit. 
    Return:
      Prints message in error
    """
    if x!=None and y!= None:
        return self.__get_a(x,y)
    elif y!=None and z!= None:
        return self.__get_a(y,z)
    elif z!=None and x!= None:
        return self.__get_a(z,x)
    else:
      print "You did something wrong...don't declare all 3 variables"
      return -1

  def __get_a(self, indep, dep):
    """ Does all calculations to get 2D line of best fit
    Args:
      indep: The variable on horrizontal axis, independent
      dep: Variable on vertical axis, dependent
    Return
      Two values used to calculate line of best fit. a0 and a1
    """
    indep_av = 0
    for val in indep: 
      indep_av += val
    indep_av = indep_av / self.__n

    dep_av = 0
    for val in dep: 
      dep_av += val
    dep_av = dep_av / self.__n

    sum_di = 0
    for i in range(0,len(indep)):
      sum_di += dep[i]*indep[i]

    sum_d = 0
    for val in dep:
      sum_d += val

    sum_i = 0
    for val in indep:
      sum_i += val

    sum_i2 = 0
    for val in indep:
      sum_i2 += math.pow(val,2)

    a1 = ((self.__n*sum_di) - sum_i*sum_d)/((self.__n*sum_i2) - math.pow(sum_i,2))
    a0 = dep_av - a1*indep_av
    
    return (a0,a1)

  def __getX(self):
    x = []
    for point in self.__points:
      x.append(point.get_x())
    return x

  def __getY(self):
    y = []
    for point in self.__points:
      y.append(point.get_y())
    return y

  def __getZ(self):
    z = []
    for point in self.__points:
      z.append(point.get_z())
    return z

def main():
  l = Lobf("./lobf-test1.txt")
  

if __name__=='__main__':
  main()
