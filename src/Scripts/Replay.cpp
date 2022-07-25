#include "Scripts.h"

  int ReplayMotion(LWR& lwr, std::string log_file_name, ControlMode mode) {
    int i, err_val = 0;

    //Logger
    MotionParser mp = MotionParser();
    mp.SetInputFile(log_file_name);
    vector<vector<float>> cart_path;
    vector<vector<float>> joint_path;

    switch (mode) {
    case CARTESIAN:
      // parse log file for cartesian path
      err_val = mp.ParseMotion(NULL, &cart_path); // #TODO
      if (err_val != EOK) {
        printf("ERROR, could not parse input file");
        break;
      }

      // move to starting position
      printf("MOVING to starting position.\n");
      err_val = lwr.MoveToCartesianPose(cart_path[0]);
      if (err_val != EOK) {
        lwr.fri_->printf("ERROR, cannot move robot to position\n");
      }

      // setup cartesian impedance control mode with high stiffness low damping
      err_val = lwr.StartCartesianImpedanceControlMode(
        lwr.CARTESIAN_STIFFNESS_HIGH, lwr.CARTESIAN_DAMPING_LOW, lwr.CARTESIAN_TORQUE_NONE);
      if (err_val != EOK) {
        printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
        break;
      }

      // send cartesian poses from path to replay
      for (vector<float> cart_pose : cart_path) {
        lwr.fri_->WaitForKRCTick();
        if (!lwr.fri_->IsMachineOK()) {
          fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
          err_val = Errors::ERROR_MACHINE_NOT_OKAY;
          break;
        }
        for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
          lwr.pose_cmd[i] = cart_pose[i];
        }
        lwr.fri_->SetCommandedCartPose(lwr.pose_cmd);
      }
      break;

    case JOINT:
      // parse log file for joint path
      err_val = mp.ParseMotion(&joint_path, NULL);
      if (err_val != EOK) {
        printf("ERROR, could not parse input file");
        break;
      }

      // move to starting position
      printf("MOVING to starting position.\n");
      err_val = lwr.MoveToJointPosition(joint_path[0], LWR::AngleUnit::DEG);
      if (err_val != EOK) {
        lwr.fri_->printf("ERROR, cannot move robot to position\n");
      }

      // setup joint impedance control mode with mid level stiffness and damping
      err_val = lwr.StartJointPositionControlMode(
        lwr.JOINT_STIFFNESS_HIGH, lwr.JOINT_DAMPING_LOW, lwr.JOINT_TORQUE_NONE);
      if (err_val != EOK) {
        printf("ERROR, could not start robot in Joint Impedance Control Mode\n");
        break;
      }

      // send joints from path to replay
      for (vector<float> joint_positions : joint_path) {
        lwr.fri_->WaitForKRCTick();
        if (!lwr.fri_->IsMachineOK()) {
          fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
          err_val = Errors::ERROR_MACHINE_NOT_OKAY;
          break;
        }
        for (i = 0; i < NUMBER_OF_JOINTS; ++i) {
          lwr.joint_msr[i] = static_cast<float>(RAD(joint_positions[i]));
        }
        lwr.fri_->SetCommandedJointPositions(lwr.joint_msr);
      }
      break;
    }
    return err_val;
  }
  