# Desctiption
Graph generation from log files

# Input
Two input files:

  - Path
  - Log

## Path
Fx, Fy, Fz, Mx, My, Mz Dx, Dy, Dz, Rx, Ry, Rz

## Log
LC Dsr, LC Msr, Position Dsr, Position Msr, PID, PID Setpoint, Next Position

# Output
  - Forces plot (3D, Fx, Fy, Fz)
  - Position plot (3D, Dx, Dy, Dz)
  - PID plot (PIDx, PIDy, PIDz)

# Future
Connection with RPC with buffered window.
