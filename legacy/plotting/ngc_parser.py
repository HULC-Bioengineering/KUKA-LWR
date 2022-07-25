from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys, getopt


HEADER_END_FLAG = 'T'
TOOLPATH_START_FLAG = 'G04 P3'
COLORS = ['b', 'r']
def parse_arguments(argv):
  output_file = None
  try:
    opts, args = getopt.getopt(argv,"hi:o:x:y:z:p:",["ifile=","ofile=", "x=", "y=", "z=", "p="])
  except getopt.GetoptError:
    print('gcode_parser.py -i <inputfile> -o <outputfile> -x <discretization x>, -y <discretization y>, -z <discretization z>, -p <plot (y/n)>')
    sys.exit(2)

  # defaults
  dx = 1
  dy = 1
  dz = 0.5
  plot = False

  for opt, arg in opts:
    if opt == '-h':
      print('gcode_parser.py -i <inputfile> -o <outputfile> -dx <Discretization X>, -dy <Discretization Y>, -dz <Discretization Z>, <plot (y/n)>')
      sys.exit()
    elif opt in ("-i", "--ifile"):
      input_file = arg
    elif opt in ("-o", "--ofile"):
      output_file = arg
    elif opt in ("-x", "--x"):
      dx = arg
    elif opt in ("-y", "--y"):
      dy = arg
    elif opt in ("-z", "--z"):
      dz = arg
    elif opt in ("-p", "--p"):
      plot = True
  return input_file, output_file, [float(dx), float(dy), float(dz)], plot

def discretize_point(p0, p1, d):
  disc_path = [p0]
  for j in [0,1,2]:
    step = p1[j] - p0[j]
    if step < 0 and d[j] > 0:
      d[j] = d[j] * -1
    elif step > 0 and d[j] < 0:
      d[j] = d[j] * -1
    for _ in range(0, int(step/d[j])):
      if j == 0:
        element = [d[j] + disc_path[-1][0], disc_path[-1][1], disc_path[-1][2]] 
      if j == 1:
        element = [disc_path[-1][0], d[j] + disc_path[-1][1], disc_path[-1][2]] 
      if j == 2:
        element = [disc_path[-1][0], disc_path[-1][1], d[j] + disc_path[-1][2]] 
      disc_path.append(element)
  return disc_path[1:]


def discretize_path(path, d):
  disc_path = [[path[0][0], path[0][1], path[0][2], 0]]
  for i in range(0, path[:,0].size - 1, 1):
    elements = discretize_point(path[i], path[i+1], d)
    for element in elements:
      disc_path.append([element[0], element[1], element[2], 1])
    disc_path.append([path[i+1][0], path[i+1][1], path[i+1][2], 0])
  return np.asarray(disc_path, dtype=np.float64)

def parse_command_motion(line, path=None):
  end = False
  command = line.split(' ')
  #print('Command: ' + ' '.join(command))
  point = [float('inf'), float('inf'), float('inf')]

  if command[0] == 'G1':
    #print('Identified: G1 command')
    if path:
      point = [path[-1][0], path[-1][1], float(command[1][1:])]
    else:
      point = [0, 0, float(command[1][1:])]

  elif command[0] == 'G0':
    #print('Identified: G0 command')
    if path:
      point = [path[-1][0], path[-1][1], float(command[1][1:])]
    else:
      point = [0, 0, float(command[1][1:])]

  elif not command[0]:
    #print('Identified: Continuation')
    params = command[1:]
    if path:
      point = [float('inf'), float('inf'), path[-1][2]]
    else:
      point = [float('inf'), float('inf'), 0]

    if len(params) != 2:
      axis = params[0][0]
      movement = float(params[0][1:])
      if axis == 'X':
        point[0] = movement
      else: # assuming Y
        point[1] = movement
    # X then Y
    else:
      point[0] = float(params[0][1:])
      point[1] = float(params[1][1:])

  elif command[0] == 'M5':
    #print('Identified: End Path')
    end = True

  #else:
  #print('Unknown')

  if path:
    if point[0] == float('inf'):
      point[0] = path[-1][0]
    if point[1] == float('inf'):
      point[1] = path[-1][1]
    if point[2] == float('inf'):
      point[2] = path[-1][2]
  else:
    if point[0] == float('inf'):
      point[0] = 0
    if point[1] == float('inf'):
      point[1] = 0
    if point[2] == float('inf'):
      point[2] = 0

  return point, end

# Returns:
def parse_tool_path(input_file):
  # open for reading
  gcode_file = open(input_file, 'r+')
  path = []

  # start of the motion path (skip file  header information)
  while not gcode_file.readline().rstrip('\n').startswith(HEADER_END_FLAG):
    pass

  # get z
  start_z, _ = parse_command_motion(gcode_file.readline().rstrip('\n'))

  #   Start of the tool path commands.
  #   Ignore for now but you can pass tool specific parameters
  #   
  #   G04 P3 => [G04 P3].split(' ') = [COMMAND, ELEMENT]
  #   E.g. 
  #     G1 Z0.0000
  #       Y13.6771
  #       X10.0000
  #     G0 Z2.0000 (max tool return height)
  while not gcode_file.readline().rstrip('\n').startswith(TOOLPATH_START_FLAG):
    pass
    
  # first line is absolute position X Y
  start_xy, _ = parse_command_motion(gcode_file.readline().rstrip('\n'))
  path.append([start_xy[0], start_xy[1], start_z[2]])

  # get rest of points till stop
  for line in gcode_file:
    point, end = parse_command_motion(line.rstrip('\n'), path)
    if end:
      break

    path.append(point)

  return np.asarray(path, dtype=np.float64)

def scrub_tool_path(path):
  # normalize
  path[:,0] -= path[:,0][0]
  path[:,1] -= path[:,1][0]
  path[:,2] -= path[:,2][0]

  # remove duplicates
  cut_list = []
  for i in range(0, path[:,0].size - 1):
    if abs(abs(path[i][0]) - abs(path[i+1][0])) == 0 and abs(abs(path[i][1]) - abs(path[i+1][1])) == 0 and abs(abs(path[i][2]) - abs(path[i+1][2])) == 0:
      cut_list.append(i)
  return np.delete(path,cut_list, axis=0), len(cut_list)

def plot_tool_path(path):
  fig = plt.figure()
  colors = True
  try:
    path[0][3]
  except IndexError:
    colors = False

  ax = fig.add_subplot( 111, projection="3d" )
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_zlim3d(-0.5, 0)
  plt.show(block=False) 
  if path[:,0].size < 20000:
    for point in path:
      if colors:
        ax.scatter(point[0], point[1], point[2], zdir='z', color=COLORS[int(point[3])])
      else:
        ax.scatter(point[0], point[1], point[2], zdir='z', color=COLORS[0])
      plt.draw()
      plt.pause(0.1)
  ax.plot(path[:,0], path[:,1], path[:,2], zdir='z', color='green')
  plt.show(block=True)

def force_uniaxial_motion(path, d):
  uniaxial_path = [path[0]]
  for i in range(0, path[:,0].size -1, 1):
    diff = [abs(abs(path[i][0]) - abs(path[i+1][0])), abs(abs(path[i][1]) - abs(path[i+1][1])), abs(abs(path[i][2]) - abs(path[i+1][2]))]
    if diff.count(0) < 2:
      # Z then Y then X order     
      uniaxial_path += discretize_point(path[i], [path[i][0], path[i][1], path[i+1][2]], d)
      uniaxial_path.append([path[i][0], path[i][1], path[i+1][2]])
      uniaxial_path += discretize_point([path[i][0], path[i][1], path[i+1][2]], [path[i][0], path[i+1][1], path[i+1][2]], d)
      uniaxial_path.append([path[i][0], path[i+1][1], path[i+1][2]])
      uniaxial_path += discretize_point([path[i][0], path[i+1][1], path[i+1][2]], [path[i+1][0], path[i+1][1], path[i+1][2]], d)
      uniaxial_path.append([path[i+1][0], path[i+1][1], path[i+1][2]])
    else:
      uniaxial_path.append(path[i+1])
  return np.asarray(uniaxial_path, dtype=np.float64)


def main(argv):
  input_file, output_file, d, plot = parse_arguments(argv)

  # parse tool path
  #print('Parsing tool path')
  path = parse_tool_path(input_file)

  # remove duplicates
  #print('Removing duplicates')
  path, num_duplicates = scrub_tool_path(path)
  #print('Removed %d duplicates' % num_duplicates)

  #print ('Splitting into uni-axial motion')
  path = force_uniaxial_motion(path, d)

  # discretize path
  #print ('Discretizing max 1mm path points')
  path = discretize_path(path, d)
    
  # second removal of duplicate points
  path, num_duplicates = scrub_tool_path(path)
  
  # plot
  #print ('Displaying plot')
  if (plot):
    plot_tool_path(path)
  
  f = open(output_file, 'w')
  for point in path:
    print('%.5f,%.5f,%.5f' % (point[0], point[1], point[2]))
    print('%.5f,%.5f,%.5f' % (point[0], point[1], point[2]), file=f)
  f.close()

if __name__=='__main__':
  main(sys.argv[1:])