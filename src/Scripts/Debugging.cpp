#include "Scripts.h"

  void Debugging(LWR& lwr, Nano25E& load_cell) {

    // set robot in free movement mode
    int err_val = EOK;
      err_val = lwr.StartCartesianImpedanceControlMode(
        lwr.CARTESIAN_STIFFNESS_LOW, lwr.CARTESIAN_DAMPING_LOW, lwr.CARTESIAN_TORQUE_NONE);
      if (err_val != EOK) {
        DBGPRINT("ERROR, could not start in Cartesian Impedance Control Mode\n");
        return;
      }


  }