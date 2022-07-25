#include "Scripts.h"

int FreeMovementPoseLoadRecord(LWR& lwr, Nano25E& lc, string output_file,
  bool restrictRotation) {
  unsigned int err_val = SUCCESS;
  char stop_char;  // for key press
  int stop_key = 27;  // ESC
  load load_val;

  const float LOAD_CELL_BASELINE_BIAS[LOAD_CELL_DOF] = {
    4.16222143173218f, 3.50110197067261f, -19.2817325592041f,
    2.74187064170837f, 0.786752343177795f, 0.95879191160202f
  };

  // debug log
  ofstream os(output_file);
  float load_cell_loads[6] = { 0, 0, 0, 0, 0, 0 };

  // calibrate, transform, start load cell
  lc.Initialize(
    Config::Filepath::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION,
    Config::SAMPLE_RATE,
    Config::LOADCELL_CHANNEL.c_str());

  // set robot in free movement mode
  err_val = lwr.StartCartesianImpedanceControlMode(
    lwr.CARTESIAN_STIFFNESS_HIGH, lwr.CARTESIAN_DAMPING_LOW, lwr.CARTESIAN_TORQUE_NONE);
  if (err_val != EOK) {
    printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
    return err_val;
  }

  // get starting pose for height and rotation
  lwr.fri_->GetMeasuredCartPose(lwr.pose_cmd);

  unsigned int i = 0;
  fprintf(stdout, "Press ESC to End\n");
  while (1) {
    // exit if esc has been hit
    if (_kbhit()) {
      stop_char = _getch();
      if (stop_char == stop_key) {
        break;
      }
    }
    lwr.fri_->WaitForKRCTick();
    if (!lwr.fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return Errors::ERROR_MACHINE_NOT_OKAY;
    }
    // get loads, joint positions, and pose
    lwr.fri_->GetMeasuredJointPositions(lwr.joint_msr);
    lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
    lc.GetLoads(load_cell_loads);
    Utils::SetLoad(load_cell_loads, load_val);

    // log
    load_val.t = i++;
    // subtract unloaded val
    load_val.x -= LOAD_CELL_BASELINE_BIAS[0];
    load_val.y -= LOAD_CELL_BASELINE_BIAS[1];
    load_val.z -= LOAD_CELL_BASELINE_BIAS[2];
    load_val.mx -= LOAD_CELL_BASELINE_BIAS[3];
    load_val.my -= LOAD_CELL_BASELINE_BIAS[4];
    load_val.mz -= LOAD_CELL_BASELINE_BIAS[5];
    Logging::WriteLoadAndPosition(os, load_val, lwr.pose_msr);

    if (restrictRotation) {
      // set x, y leavee rotation and height fixed to starting values
      lwr.pose_cmd[3] = lwr.pose_msr[3];
      lwr.pose_cmd[7] = lwr.pose_msr[7];
      lwr.pose_cmd[11] = lwr.pose_msr[11];

      lwr.fri_->SetCommandedCartPose(lwr.pose_cmd);
    }
    else {
      lwr.fri_->SetCommandedCartPose(lwr.pose_msr);
    }
  }
  lc.Stop();
  return SUCCESS;
}