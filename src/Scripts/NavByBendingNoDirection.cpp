#include "Scripts.h"

namespace NavByBendingNoDirection {

  ///////////////////////////////////////////////////////////////////////////////
  ////                         NAV BY BENDING HELPERS                        ////
  ///////////////////////////////////////////////////////////////////////////////

  // Apply the transformation from the measured load cell values to the current
  // pose resulting in the next step / desired pose.
  //
  // Parameters:
  //   pose: Current Cartesian pose.
  //   loads: Load cell force/torque values transformed to the tools
  //          coordinate system located at the TCP and converted to position
  //          and rotational changes (via PID).
  //   next_pose: pointer to an array which is set to T ti+1 wrt base.
  void ForceMovement(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    const double(&loads)[LOAD_CELL_DOF], float* next_pose) {
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> result_pose_matrix;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;

    // create eigen matrix structure from the current cartesian pose
    current_pose_matrix <<
      pose[0], pose[1], pose[2], pose[3],
      pose[4], pose[5], pose[6], pose[7],
      pose[8], pose[9], pose[10], pose[11],
      0, 0, 0, 1;

    // create eigen matrix from load cell values using
    // fixed X-Y-Z rotation sequence. Load cell values should already be
    // transformed to the tools coordinate system and located at the TCP
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
    float sA = static_cast<float>(sin(RAD(loads[3])));
    float sB = static_cast<float>(sin(RAD(loads[4])));
    float sG = static_cast<float>(sin(RAD(loads[5])));
    float cA = static_cast<float>(cos(RAD(loads[3])));
    float cB = static_cast<float>(cos(RAD(loads[4])));
    float cG = static_cast<float>(cos(RAD(loads[5])));
    lc_matrix <<
      cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), loads[0],
      sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), loads[1],
      -sB, cB*sG, cB*cG, loads[2],
      0, 0, 0, 1;

    // Find base, ti+1
    // base     ti       base
    //     T      T  =       T
    //   ti   ti+1       ti+1
    result_pose_matrix = current_pose_matrix * lc_matrix;

    // set next cartesian pose for FRI communication
    for (unsigned int i = 0; i < 12; ++i) {
      next_pose[i] = result_pose_matrix((i / 4), (i % 4));
    }
  }

  // Safety check if current pose is within the maxiumum bounds of the desired
  // position. The check current uses positonal values and no rotations.
  //
  // Parameters:
  //   dsr_position: Vector containing the desired [X, Y, Z] position.
  //   msr_pose: Current measured pose.
  //   max_bounds: Vector containing the maximum X, Y, Z positional offset.
  //               Values outside this range are considered out of bounds.
  //
  // Returns:
  //  true if all X, Y, Z values of the current measured pose are within the
  //  maximum bounds, else false.
  bool InBounds(const vector<double>& dsr_position, const float(&msr_pose)[12],
    const vector<double>& max_bounds) {
    if (abs(dsr_position[0] - msr_pose[3]) > max_bounds[0] ||
      abs(dsr_position[1] - msr_pose[7]) > max_bounds[1] ||
      abs(dsr_position[2] - msr_pose[11]) > max_bounds[2]) {
      DBGPRINT("%f, %f, %f\n",
        abs(dsr_position[0] - msr_pose[3]),
        abs(dsr_position[1] - msr_pose[7]),
        abs(dsr_position[2] - msr_pose[11]));
      return false;
    }
    return true;
  }

  ///////////////////////////////////////////////////////////////////////////////
  ////                               NAV BY                                  ////
  ///////////////////////////////////////////////////////////////////////////////

  // Main algorithm as described in thesis. Move to a pose which satasfies
  // a bending loads point. Once a point is found, record pose.
  // Contains the following main subsystems:
  //   - Bounding box
  //   - Timeout between points
  //   - Output file static location
  //   - Output statistics file
  //   - Delay between points option
  //   - 3x parrell PID's (X,Y,Z) 
  //
  // Parameters:
  //   lwr: KUKA LWR.
  //   load_cell: Nano25E load cell.
  //   follow_loads_path: Input file containing loads and positions.
  //   output_filename: Output file.
  //   point_delay: Delay between each path point.
  int NavByBendingForces(LWR& lwr, Nano25E& load_cell,
    const string follow_loads_path, const string output_filename,
    const unsigned int point_delay) {
    // maximum number of steps between each path point iteration
    const unsigned int TIMEOUT_COUNT = 1500;
    // timestep for the PID controllers
    const double PID_TIMESTEP = 0.002;
    // maximum PID controller output per iteration. Note: 0.001 is the LWR max
    const double PID_MAX_TRANSLATION = 0.0005;
    // maximum load cell force value
    const double PID_MAX_FORCE = 25.0;
    // X, Y, Z safety boundary
    const vector<double> SAFETY_BOUNDARY = { 0.005, 0.005, 0.003 };

    const string LOG_PATH_BASE =
      "C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\";
    const string LOG_PATH_FILENAME = LOG_PATH_BASE +
      output_filename + CSV;
    const string STATISTICS_FILENAME = LOG_PATH_BASE +
      output_filename + "_results" + CSV;
    const string HEADER = "Dsr Loads, , , , , , Msr Loads, , , , , , "
      "Dsr Pos, , , Msr Pos, , , Tool PID's, , , PID Setpoint, , , "
      "Next Pos, , , Direction\n";
    const string HEADER2 = "Fx, Fy, Fz, Mx, My, Mz, Fx, Fy, Fz, Mx, My, Mz, "
      "Px, Py, Pz, Px, Py, Pz, Px, Py, Pz, dX, dY, dZ, Sign, Axis\n";

    vector<PID> pids = {
      PID(50, 0, 0, PID_TIMESTEP),
      PID(50, 0, 0, PID_TIMESTEP),
      PID(50, 0, 0, PID_TIMESTEP) };

    // working variables
    unsigned int err_val = 0;
    unsigned int path_point = 0;
    unsigned int timeout_counter = 0;
    float load_cell_loads[6] = { 0, 0, 0, 0, 0, 0 };
    vector<vector<float>> found_points_pose;
    vector<double> pid_step_computed = { 0, 0, 0 };

    // open log file and write header
    ofstream os(LOG_PATH_FILENAME);
    os << HEADER << HEADER2;

    // set PID input limits to max acceptable force
    pids[0].setInputLimits(-PID_MAX_FORCE, PID_MAX_FORCE);
    pids[1].setInputLimits(-PID_MAX_FORCE, PID_MAX_FORCE);
    pids[2].setInputLimits(-PID_MAX_FORCE, PID_MAX_FORCE);

    // set PID output limits to max translations for step
    pids[0].setOutputLimits(-PID_MAX_TRANSLATION, PID_MAX_TRANSLATION);
    pids[1].setOutputLimits(-PID_MAX_TRANSLATION, PID_MAX_TRANSLATION);
    pids[2].setOutputLimits(-PID_MAX_TRANSLATION, PID_MAX_TRANSLATION);

    // get loads in path
    vector<vector<double>> loads;
    vector<vector<double>> positions;
    if (Reader::GetLoadsFromCSV(follow_loads_path, loads)) {
      return 1; // invalid input
    }
    if (Reader::GetPositionsFromCSV(follow_loads_path, positions)) {
      return 1; // invalid input
    }

    // calibrate, transform, start load cell
    if (load_cell.Initialize(
      Config::LOAD_CELL_CALIBRATION_PATH.c_str(),
      Config::LOAD_CELL_TRANSFORMATION,
      Config::SAMPLE_RATE,
      Config::LOADCELL_CHANNEL.c_str())) {
      return 2; // load cell initialization error
    }
    delay(2000);

    // start robot in cartesian impedance control mode
    float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
    lwr.fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    err_val = lwr.StartCartesianImpedanceControlMode(
      lwr.CARTESIAN_STIFFNESS_HIGH,
      lwr.CARTESIAN_DAMPING_LOW,
      est_ft);
    if (err_val != SUCCESS) {
      printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
      load_cell.Stop();
      return 3; // cartesian impedance control mode error
    }

    // set PID set points to first point in path
    pids[0].setSetPoint(loads[1][0]);
    pids[1].setSetPoint(loads[1][1]);
    pids[2].setSetPoint(loads[1][2]);

    // get starting process value
    load_cell.GetLoads(load_cell_loads);
    pids[0].setProcessValue(load_cell_loads[0]);
    pids[1].setProcessValue(load_cell_loads[1]);
    pids[2].setProcessValue(load_cell_loads[2]);

    DBGPRINT("Starting Nav By Bending Forces");
    while (path_point < loads.size() - 1) {

      // resolve current path point
      while (1) {
        lwr.fri_->WaitForKRCTick();

        // FRI connection check
        if (!lwr.fri_->IsMachineOK()) {
          printf("ERROR, the machine is not ready anymore\n");
          for (auto pose : found_points_pose) {
            DBGPRINT("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
          }
          load_cell.Stop();
          os.close();
          return 4;  // error machine not okay
        }

        // set FRI force term to self
        lwr.fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
        lwr.fri_->SetCommandedCartForcesAndTorques(est_ft);

        // timeout if in sink
        if (timeout_counter++ >= TIMEOUT_COUNT) {
          load_cell.Stop();
          os.close();
          printf("Sink!\n");
          return Errors::ERROR_TIMEOUT_COUNT;  // error sink
        }

        // get process value
        load_cell.GetLoads(load_cell_loads);
        pids[0].setProcessValue(load_cell_loads[0]);
        pids[1].setProcessValue(load_cell_loads[1]);
        pids[2].setProcessValue(load_cell_loads[2]);

        // get pose
        lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);

        // max bounds timeout check
        if (!InBounds(positions[path_point], lwr.pose_msr, SAFETY_BOUNDARY)) {
          load_cell.Stop();
          printf("Out of bounds"
            "Dsr: (%.5f, %.5f, %.5f) vs. Msr: (%.5f, %.5f, %.5f)\n",
            positions[path_point][0], positions[path_point][1],
            positions[path_point][2], lwr.pose_msr[3], lwr.pose_msr[7],
            lwr.pose_msr[11]);
          return Errors::ERROR_OUT_OF_BOUNDS; // error out of bounds
        }

        // compute PID error correction value using deadband
        DeadbandCompute(movement_direction, pids, deadbands, pid_step_computed);

        // log loads
        os <<
          loads[path_point + 1][0] << "," <<
          loads[path_point + 1][1] << "," <<
          loads[path_point + 1][2] << "," <<
          loads[path_point + 1][3] << "," <<
          loads[path_point + 1][4] << "," <<
          loads[path_point + 1][5] << ",";
        LogLoad(os, load_cell_loads, PrettyPrint::ENDLINE::ENDCOMMA);

        // log pose
        os <<
          positions[path_point + 1][0] << "," <<
          positions[path_point + 1][1] << "," <<
          positions[path_point + 1][2] << ",";
        LogPose(os, lwr.pose_msr, PrettyPrint::ENDLINE::ENDCOMMA);

        // log PID error correction value
        os <<
          pid_step_computed[0] << "," <<
          pid_step_computed[1] << "," <<
          pid_step_computed[2] << ",";
        os <<
          pids[0].getSetPoint() << "," <<
          pids[1].getSetPoint() << "," <<
          pids[2].getSetPoint() << ",";

        // ensure direction of travel always moving forwards
        switch (movement_direction.axis) {
        case X: // x #TODO local
          if (abs(pid_step_computed[2]) <= PID_MIN_PATH_TRANSLATION + 0.0001) {
            flag_path_direction_slowed = true;
          }
          break;
        case Y:
          if (abs(pid_step_computed[1]) <= PID_MIN_PATH_TRANSLATION) {
            flag_path_direction_slowed = true;
          }
          break;
        case Z:  // z #TODO local
          if (abs(pid_step_computed[0]) <= PID_MIN_PATH_TRANSLATION - 0.0001) {
            flag_path_direction_slowed = true;
          }
          break;
        default:
          printf("Incorrect direction determined");
          break;
        }
        if (flag_path_direction_slowed) {
          flag_path_direction_slowed = false;
          break;
        }

        // convert PID force from tool frame to global coordinate system
        ForceMovement(lwr.pose_msr,
        { pid_step_computed[0],
        pid_step_computed[1],
        pid_step_computed[2],
        0, 0, 0 },
        lwr.pose_cmd);

        // latching (overrides PID in non-travel directions)  
        /*
        switch (movement_direction.axis) {
        case X:
        lwr.pose_cmd[7] = pos_y_latch;
        lwr.pose_cmd[11] = pos_z_latch;
        break;
        case Y:
        lwr.pose_cmd[3] = pos_x_latch;
        lwr.pose_cmd[11] = pos_z_latch;
        break;
        case Z:
        lwr.pose_cmd[3] = pos_x_latch;
        lwr.pose_cmd[7] = pos_y_latch;
        break;
        }
        */

        // set next pose
        lwr.fri_->SetCommandedCartPose(lwr.pose_cmd);

        // log next pose
        LogPose(os, lwr.pose_cmd, PrettyPrint::ENDLINE::ENDCOMMA);

        os << movement_direction.sign << "," << movement_direction.axis << "\n";
      }

      // found point
      LogPose(os, lwr.pose_cmd, PrettyPrint::ENDLINE::NEWLINE);
      printf("Path Point %d. @: %.2f, %.2f, %.2f\n", path_point,
        lwr.pose_msr[3] * 1000, lwr.pose_msr[7] * 1000, lwr.pose_msr[11] * 1000);
      path_point++;

      // set new set point for next points load values
      if (path_point != loads.size() - 1) {
        pids[0].setSetPoint(loads[path_point + 1][0]);
        pids[1].setSetPoint(loads[path_point + 1][1]);
        pids[2].setSetPoint(loads[path_point + 1][2]);
      }

      // reset timeout counter
      timeout_counter = 0;

      // log found point pose
      vector<float> v;
      for (unsigned int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
        v.push_back(lwr.pose_msr[i]);
      }
      found_points_pose.push_back(v);

      // delay between found points
      delay(point_delay);
    }

    // completion. Stop load cell and print found point poses
    load_cell.Stop();
    for (auto pose : found_points_pose) {
      DBGPRINT("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
    }

    // generate stats file
    Stats::GenerateNavByBendingPositionResults(found_points_pose, positions,
      STATISTICS_FILENAME);

    return SUCCESS;
  }


}