# Description
A method for determining a Caretsian cutting path from a 3D solid model. 
# Input
  - 3D model (.sldprt, .x_t, .stl)
  - Cutting tool diameter (mm)

# Output
  - List of Cartesian frames [tool wrt. base]

# Notes
  - Investigate 5-axis CNC machine tool paths
  - Typical conversion process from .stl to .gcode is: 
    - .sldprt -> *Save as* -> .stl
    - .stl -> *Slic3r* -> .gcode

