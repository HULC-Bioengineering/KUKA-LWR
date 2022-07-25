import sys
import csv
import math

_FILE_NAME = './input.stl'


def get_value(arr):
  valueX = arr[1].split('e')
  x_digit = float(valueX[0]) * float(10**int(valueX[1]))
 
  valueY = arr[2].split('e')
  y_digit = float(valueY[0]) * float(10**int(valueY[1]))

  valueZ = arr[3].split('e')
  z_digit = float(valueZ[0]) * float(10**int(valueZ[1]))
  
  return [x_digit, y_digit, z_digit]


def main():
  reader = open(_FILE_NAME, 'r')
  points = []

  for row in reader:
    if 'vertex' in row:
      row = row.strip()
      arr = row.split(' ')

      point = get_value(arr)
      point = ','.join(str(x) for x in point)
      points.append(point)

  points = list(set(points))

  g = open('output.csv', 'w')

  for point in points:
    g.write(point)
    g.write("\n")

if __name__=='__main__':
  main()
