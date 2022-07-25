#include "Scripts.h"

  int RecordMotion(LWR& lwr, std::string log_file_name, ControlMode mode,
    RecordStopCondition stop_condition) {

    int err_val = EOK;
    int duration = 30;  // 30 seconds if time option selected
    int cycles = 0;
    char stop_char;  // for key press
    int stop_key = 27;  // ESC

    // set robot in free movement mode
    if (mode == CARTESIAN) {
      err_val = lwr.StartCartesianImpedanceControlMode(
        lwr.CARTESIAN_STIFFNESS_LOW, lwr.CARTESIAN_DAMPING_LOW, lwr.CARTESIAN_TORQUE_NONE);
      if (err_val != EOK) {
        printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
        return err_val;
      }
    }
    else if (mode == JOINT) {
      err_val = lwr.StartJointImpedanceControlMode(
        lwr.JOINT_STIFFNESS_LOW, lwr.JOINT_DAMPING_LOW, lwr.JOINT_TORQUE_NONE);
      if (err_val != EOK) {
        printf("ERROR, could not start in Joint Impedance Control Mode\n");
        return err_val;
      }
    }

    // prepare unique log identifier and start logging
    DBGPRINT("Preparing to log");
    char file_identifier[10] = { "\0" };
    Utils::GenerateFileIdentifier(file_identifier);
    err_val = lwr.fri_->PrepareLogging(file_identifier);
    if (err_val != 0) {
      printf("Incorrect /etc log file location\n");
    }
    DBGPRINT("Free movement enabled");

    switch (stop_condition) {
    case A0_ANGLE:
      err_val = lwr.fri_->StartLogging();
      if (err_val != EOK) {
        printf("ERROR, cannot start data logging\n");
        return err_val;
      }
      DBGPRINT("Started logging");

      while (DEG(lwr.joint_msr[1]) < 40) {
        lwr.fri_->WaitForKRCTick();
        if (!lwr.fri_->IsMachineOK()) {
          printf("ERROR, the machine is not ready anymore\n");
          return Errors::ERROR_MACHINE_NOT_OKAY;
        }
        lwr.fri_->GetMeasuredJointPositions(lwr.joint_msr);
        lwr.fri_->SetCommandedJointPositions(lwr.joint_msr);
        lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
        lwr.fri_->SetCommandedCartPose(lwr.pose_msr);
      }
      break;

    case TIME:
      err_val = lwr.fri_->StartLogging();
      if (err_val != EOK) {
        printf("ERROR, cannot start data logging\n");
        return err_val;
      }
      DBGPRINT("Started logging");

      while (static_cast<float>(cycles)* lwr.fri_->GetFRICycleTime() < duration) {
        if (!lwr.fri_->IsMachineOK()) {
          printf("ERROR, the machine is not ready anymore\n");
          return Errors::ERROR_MACHINE_NOT_OKAY;
        }
        lwr.fri_->GetMeasuredJointPositions(lwr.joint_msr);
        lwr.fri_->SetCommandedJointPositions(lwr.joint_msr);
        lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
        lwr.fri_->SetCommandedCartPose(lwr.pose_msr);
        lwr.fri_->WaitForKRCTick();
        cycles++;
      }
      break;

    case KEY_PRESS:
      fprintf(stdout, "Press ESC to Start\n");
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
        lwr.fri_->GetMeasuredJointPositions(lwr.joint_msr);
        lwr.fri_->SetCommandedJointPositions(lwr.joint_msr);
        lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
        lwr.fri_->SetCommandedCartPose(lwr.pose_msr);
      }

      err_val = lwr.fri_->StartLogging();
      if (err_val != EOK) {
        printf("ERROR, cannot start data logging\n");
        return err_val;
      }
      DBGPRINT("Started logging");

      fprintf(stdout, "Press ESC to Exit\n");
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
        lwr.fri_->GetMeasuredJointPositions(lwr.joint_msr);
        lwr.fri_->SetCommandedJointPositions(lwr.joint_msr);
        lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
        lwr.fri_->SetCommandedCartPose(lwr.pose_msr);
      }
      break;
    }

    // stop logging
    err_val = lwr.fri_->StopLogging();
    if (err_val != EOK) {
      printf("ERROR, cannot stop data logging\n");
    }
    DBGPRINT("Stopped logging");

    // Restore rigidity
    lwr.SetCartesianParameters(lwr.CARTESIAN_STIFFNESS_HIGH, lwr.CARTESIAN_DAMPING_HIGH);

    // stop robot
    err_val = lwr.fri_->StopRobot();
    if (err_val != EOK) {
      fprintf(stderr, "An error occurred during stopping the robot...\n");
    }
    printf("STOPPED robot\n");

    // write logging file
    err_val = lwr.fri_->WriteLoggingDataFile();
    if (err_val != EOK) {
      lwr.fri_->printf("ERROR, cannot write data log.\n");
    }

    Utils::RenameLogFile(
      log_file_name, file_identifier, Config::Filepath::LOGS_PATH, Config::Extension::DAT);
    printf("WROTE log file\n");

    return(EXIT_SUCCESS);
  }
