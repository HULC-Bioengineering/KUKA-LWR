#include "Scripts.h"

int FreeCartesianMovement(LWR& lwr, int duration) {
  int err_val = 0;
  int cycles = 0;

  err_val = lwr.StartCartesianImpedanceControlMode(
    lwr.CARTESIAN_STIFFNESS_LOW, lwr.CARTESIAN_DAMPING_LOW, lwr.CARTESIAN_TORQUE_NONE);
  if (err_val != EOK) {
    printf("ERROR, could not start robot in CartImpedanceControlMode");
    return err_val;
  }

  printf("You have %d seconds to position the robot...hurry.\n", duration);
  while (static_cast<float>(cycles)* lwr.fri_->GetFRICycleTime() < duration) {
    if (!lwr.fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return Errors::ERROR_MACHINE_NOT_OKAY;
    }
    lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
    lwr.fri_->SetCommandedCartPose(lwr.pose_msr);
    lwr.fri_->WaitForKRCTick();
    cycles++;
  }

  // Restore rigidity
  lwr.fri_->SetCommandedCartStiffness(lwr.CARTESIAN_STIFFNESS_HIGH);
  lwr.fri_->SetCommandedCartDamping(lwr.CARTESIAN_DAMPING_HIGH);
  lwr.fri_->WaitForKRCTick();
  return EOK;
}