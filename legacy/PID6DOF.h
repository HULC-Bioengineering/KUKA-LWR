#ifndef PID6DOF_H
#define PID6DOF_H

#include "PID.h"

const float PID_TIMESTEP = 0.002;  // should be same as fri_init calibration val

// 6 parrell Proportional-integral-derivative controllers.
class PID6DOF {
public:
  PID x_ = PID(1200, 2, 0, PID_TIMESTEP);
  PID y_ = PID(1200, 2, 0, PID_TIMESTEP);
  PID z_ = PID(1200, 2, 0, PID_TIMESTEP);
  // ignore rotations
  PID a_ = PID(0.0, 0, 0, PID_TIMESTEP);  
  PID b_ = PID(0.0, 0, 0, PID_TIMESTEP);
  PID c_ = PID(0.0, 0, 0, PID_TIMESTEP);

  PID6DOF();
  void setSetPoint(const float(&dsr)[12]);
  void setProcessValue(const float(&msr)[12]);
  void setInputLimits(const unsigned int num, const int low, const int high);
  void setGains(const float(&gains)[6]);
  void setIntegrals(const float(&integrals)[6]);
  void compute(float* msr);
};

#endif /* PID6DOF_H */