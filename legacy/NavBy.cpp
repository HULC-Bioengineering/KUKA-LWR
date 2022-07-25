void LWR::CreateTransformationFromLoadValues(const float* scale, const float(&load_cell_forces)[LOAD_CELL_DOF], matrix<float>* lc_matrix) {
  // safety to ensure step within reachable distance bounds
  float scaled_load_cell_force_x = clip(load_cell_forces[0] * scale[0] / 1000,
    -Config::MAX_DISPLACEMENT_PER_STEP, Config::MAX_DISPLACEMENT_PER_STEP);
  float scaled_load_cell_force_y = clip(load_cell_forces[1] * scale[1] / 1000,
    -Config::MAX_DISPLACEMENT_PER_STEP, Config::MAX_DISPLACEMENT_PER_STEP);
  float scaled_load_cell_force_z = clip(load_cell_forces[2] * scale[2] / 1000,
    -Config::MAX_DISPLACEMENT_PER_STEP, Config::MAX_DISPLACEMENT_PER_STEP);

  // safety to ensure step within reachable rotation bounds
  float G = clip(load_cell_forces[3] * scale[3] / 1000,
    -Config::MAX_ROTATION_PER_STEP, Config::MAX_ROTATION_PER_STEP);
  float B = clip(load_cell_forces[4] * scale[4] / 1000,
    -Config::MAX_ROTATION_PER_STEP, Config::MAX_ROTATION_PER_STEP);
  float A = clip(load_cell_forces[5] * scale[5] / 1000,
    -Config::MAX_ROTATION_PER_STEP, Config::MAX_ROTATION_PER_STEP);

  float sA = static_cast<float>(sin(RAD(A)));
  float sB = static_cast<float>(sin(RAD(B)));
  float sG = static_cast<float>(sin(RAD(G)));
  float cA = static_cast<float>(cos(RAD(A)));
  float cB = static_cast<float>(cos(RAD(B)));
  float cG = static_cast<float>(cos(RAD(G)));

  // row 1
  lc_matrix->insert_element(0, 0, cA*cB);
  lc_matrix->insert_element(0, 1, (cA*sB*sG) - (sA*cG));
  lc_matrix->insert_element(0, 2, (cA*sB*cG) + (sA*sG));
  lc_matrix->insert_element(0, 3, scaled_load_cell_force_x);
  // row 2
  lc_matrix->insert_element(1, 0, sA*cB);
  lc_matrix->insert_element(1, 1, (sA*sB*sG) + (cA*cG));
  lc_matrix->insert_element(1, 2, (sA*sB*cG) - (cA*sG));
  lc_matrix->insert_element(1, 3, scaled_load_cell_force_y);
  // row 3
  lc_matrix->insert_element(2, 0, -sB);
  lc_matrix->insert_element(2, 1, cB*sG);
  lc_matrix->insert_element(2, 2, cB*cG);
  lc_matrix->insert_element(2, 3, scaled_load_cell_force_z);
  // row 4
  lc_matrix->insert_element(3, 0, 0);
  lc_matrix->insert_element(3, 1, 0);
  lc_matrix->insert_element(3, 2, 0);
  lc_matrix->insert_element(3, 3, 1);
}

void LWR::ForceMovement(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], const float* scale, const float(&loads)[LOAD_CELL_DOF], float* next_pose) {
  //matrix<float> result_pose_matrix(4, 4);  // intermediary matrix
  //matrix<float> current_pose_matrix(4, 4);  // current pose

  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> result_pose_matrix;
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;


  // construct matrix from current pose
  //BoostMatrix::BuildPoseMatrixFromArray(pose_msr, &current_pose_matrix);
  current_pose_matrix <<
    pose[0], pose[1], pose[2], pose[3],
    pose[4], pose[5], pose[6], pose[7],
    pose[8], pose[9], pose[10], pose[11],
    0, 0, 0, 1;

  // create load cell matrix using fixed X-Y-Z rotation sequence
  //matrix<float> lc_matrix(4, 4);
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
  CreateTransformationFromLoadValues(scale, loads, lc_matrix);
  // Find base, ti+1
  // base     ti       base
  //     T      T  =       T
  //   ti   ti+1       ti+1
  //result_pose_matrix = prod(current_pose_matrix, lc_matrix);
  result_pose_matrix = current_pose_matrix * lc_matrix;

  // set the next pose from values in the resulting matrix
  //next_pose = result_pose_matrix.data();
  next_pose[0] = result_pose_matrix(0, 0);
  next_pose[1] = result_pose_matrix(0, 1);
  next_pose[2] = result_pose_matrix(0, 2);
  next_pose[3] = result_pose_matrix(0, 3);
  next_pose[4] = result_pose_matrix(1, 0);
  next_pose[5] = result_pose_matrix(1, 1);
  next_pose[6] = result_pose_matrix(1, 2);
  next_pose[7] = result_pose_matrix(1, 3);
  next_pose[8] = result_pose_matrix(2, 0);
  next_pose[9] = result_pose_matrix(2, 1);
  next_pose[10] = result_pose_matrix(2, 2);
  next_pose[11] = result_pose_matrix(2, 3);
  // cout << result_pose_matrix;
  // PrettyPrint::PrintPose(next_pose);
  //BoostMatrix::BuildPoseArrayFromMatrix(result_pose_matrix, next_pose);
}

void LWR::ForceMovement2(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], const float* scale, const float(&loads)[LOAD_CELL_DOF], float* next_pose) {
  matrix<float> result_pose_matrix(4, 4);  // intermediary matrix
  matrix<float> current_pose_matrix(4, 4);  // current pose

  // construct matrix from current pose
  BoostMatrix::BuildPoseMatrixFromArray(pose, &current_pose_matrix);

  // create load cell matrix using fixed X-Y-Z rotation sequence
  matrix<float> lc_matrix(4, 4);
  //Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
  CreateTransformationFromLoadValues(scale, loads, &lc_matrix);
  // Find base, ti+1
  // base     ti       base
  //     T      T  =       T
  //   ti   ti+1       ti+1
  result_pose_matrix = prod(current_pose_matrix, lc_matrix);
  //result_pose_matrix = current_pose_matrix * lc_matrix;

  // set the next pose from values in the resulting matrix
  //next_pose = result_pose_matrix.data();
  BoostMatrix::BuildPoseArrayFromMatrix(result_pose_matrix, next_pose);
  //cout << result_pose_matrix;
}

void LWR::CreateTransformationFromLoadValues(const float* scale, const float(&load_cell_forces)[LOAD_CELL_DOF], Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& lc_matrix) {
  // safety to ensure step within reachable distance bounds
  float scaled_load_x = clip(load_cell_forces[0] * scale[0] / 1000,
    -Config::MAX_DISPLACEMENT_PER_STEP, Config::MAX_DISPLACEMENT_PER_STEP);
  float scaled_load_y = clip(load_cell_forces[1] * scale[1] / 1000,
    -Config::MAX_DISPLACEMENT_PER_STEP, Config::MAX_DISPLACEMENT_PER_STEP);
  float scaled_load_z = clip(load_cell_forces[2] * scale[2] / 1000,
    -Config::MAX_DISPLACEMENT_PER_STEP, Config::MAX_DISPLACEMENT_PER_STEP);

  // safety to ensure step within reachable rotation bounds
  float G = clip(load_cell_forces[3] * scale[3] / 1000,
    -Config::MAX_ROTATION_PER_STEP, Config::MAX_ROTATION_PER_STEP);
  float B = clip(load_cell_forces[4] * scale[4] / 1000,
    -Config::MAX_ROTATION_PER_STEP, Config::MAX_ROTATION_PER_STEP);
  float A = clip(load_cell_forces[5] * scale[5] / 1000,
    -Config::MAX_ROTATION_PER_STEP, Config::MAX_ROTATION_PER_STEP);

  float sA = static_cast<float>(sin(RAD(A)));
  float sB = static_cast<float>(sin(RAD(B)));
  float sG = static_cast<float>(sin(RAD(G)));
  float cA = static_cast<float>(cos(RAD(A)));
  float cB = static_cast<float>(cos(RAD(B)));
  float cG = static_cast<float>(cos(RAD(G)));

  lc_matrix <<
    cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), scaled_load_x,
    sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), scaled_load_y,
    -sB, cB*sG, cB*cG, scaled_load_z,
    0, 0, 0, 1;
}

bool InBounds(vector<double> dsr_position, float (&msr_pose)[12], const float* max_bounds) {
  if (abs(dsr_position[1] - msr_pose[3]) > max_bounds[0] ||
    abs(dsr_position[2] - msr_pose[7]) > max_bounds[1] ||
    abs(dsr_position[3] - msr_pose[11]) > max_bounds[2]) { 
    printf("%f, %f, %f\n", abs(dsr_position[1] - msr_pose[3]), abs(dsr_position[2] - msr_pose[7]), abs(dsr_position[3] - msr_pose[11]));
    return false;
  }
  return true;
}

int LWR::BoundingBoxNavByBendingForces(string follow_loads_path, string output_position_filename,
  const float* max_bounds, vector<float>& gains, Vector3d& avgs, Vector3d& stdevs) {
  Logging::Logger log = Logging::Logger();

  load current_load;
  unsigned int path_point = 0;
  int err_val = 0;
  load boundary_threshold = { 0, 0.1f, 0.1f, 0.1f, 0.2f, 0.2f, 0.2f };
  float arr[6];
  arr[0] = gains[0]; arr[1] = gains[1]; arr[2] = gains[2];
  arr[3] = -0.00000000001; arr[4] = -0.00000000001; arr[5] = -0.00000000001;
  vector<vector<float>> found_points_pose;
  unsigned int TIMEOUT_COUNT = 2000;

  // get loads in path
  vector<load> loads;
  vector<vector<double>> positions;
  log.GetLoadsFromCSV(follow_loads_path, loads);
  log.GetPositionsFromCSV(follow_loads_path, positions);

  // calibrate, transform, start load cell
  load_cell_->Initialize(Config::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION, Config::SAMPLE_RATE, Config::LOADCELL_CHANNEL.c_str());

  // start robot in cartesian impedance control mode
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    load_cell_->Stop();
    return err_val;
  }
  // set starting load to find
  load find_load = loads[path_point];
  vector<double> find_pos = positions[0];

  // debug log
  string dbg_base = "C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\prelim\\logged";
  string dbg_name = dbg_base + std::to_string(path_point) + CSV;
  ofstream os(dbg_name);
  LogLoad(os, find_load);  // #TODO
  log.LogHeader(os);

  unsigned int timeout_counter = 0;
  DBGPRINT("Starting Nav By Bending Forces");
  while (path_point < loads.size()) {
    fri_->WaitForKRCTick();

    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      DBGPRINT("KILLED AT: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", load_cell_loads[0], load_cell_loads[1], load_cell_loads[2], load_cell_loads[3], load_cell_loads[4], load_cell_loads[5]);
      for (auto pose : found_points_pose) {
        printf("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
      }
      load_cell_->Stop();
      return ERROR_MACHINE_NOT_OKAY;
    }

    // timeout if in sink
    if (timeout_counter++ >= TIMEOUT_COUNT) {
      load_cell_->Stop();
      printf("Sink!\n");
      return Errors::ERROR_TIMEOUT_COUNT;
    }

    // get current loadcell forces
    load_cell_->GetLoads(load_cell_loads);
    LogLoad(os, load_cell_loads, PrettyPrint::ENDCOMMA);

    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    // subtract desired point from lc forces (shift number line)
    Utils::SetLoad(load_cell_loads, current_load);
    Utils::SetDesiredPosition(load_cell_loads, find_load);
    LogLoad(os, load_cell_loads, PrettyPrint::ENDCOMMA);

    // get current pose
    fri_->GetMeasuredCartPose(pose_msr);
    LogPose(os, pose_msr, PrettyPrint::ENDCOMMA);

    // max bounds timeout check
    if (!InBounds(find_pos, pose_msr, max_bounds)) {
      // #TODO invalidate/remove output file upon failure
      load_cell_->Stop();
      printf("Out of bounds!\n");
      return Errors::ERROR_OUT_OF_BOUNDS;  // out of max bounds error
    }

    // in loads position
    if (Utils::ReachedAcceptableLoadPosition(current_load, find_load, boundary_threshold)) {
      DBGPRINT("DBG Point %d. @: %.2f, %.2f, %.2f\n", path_point, pose_msr[3] * 1000, pose_msr[7] * 1000, pose_msr[11] * 1000);
      find_load = loads[path_point];
      find_pos = positions[path_point];
      path_point++;
      timeout_counter = 0;

      vector<float> v;
      for (unsigned int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
        v.push_back(pose_msr[i]);
      }
      found_points_pose.push_back(v);

      // new dbg file
      dbg_name = dbg_base + std::to_string(path_point) + CSV;
      os.close();
      os.open(dbg_name);
      LogLoad(os, find_load);
      log.LogHeader(os);
      delay(500);
    }

    // move to loads position
    else {
      // calculate how much to move based on forces to reach desired position
      if (timeout_counter % 20 == 0) {
        // DBGPRINT("Point %d found @: %.2f, %.2f, %.2f\n", path_point, pose_msr[3] * 1000, pose_msr[7] * 1000, pose_msr[11] * 1000);
      }
      ForceMovement(pose_msr, arr, load_cell_loads, pose_cmd);
      fri_->SetCommandedCartPose(pose_cmd);
      LogPose(os, pose_cmd);
    }
  }

  load_cell_->Stop();
  for (auto pose : found_points_pose) {
    printf("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
  }
  Stats::GenerateNavByBendingPositionResults(found_points_pose, positions,
    output_position_filename,
    avgs, stdevs);
  return SUCCESS;
}

int LWR::BoundingBoxNavByBendingForcesv2(string follow_loads_path, string output_position_filename,
  const float* max_bounds, vector<float>& gains, vector<float>& integrals, Vector3d& avgs, Vector3d& stdevs) {
  Logging::Logger log = Logging::Logger();
  unsigned int TIMEOUT_COUNT = 4000000000;
  vector<vector<float>> found_points_pose;
  unsigned int path_point = 0;
  int err_val = 0;
  float boundary_threshold[6] = { 0.2f, 0.001f, 0.2f, 0.2f, 0.2f, 0.2f };
  float computed[6] = {0,0,0,0,0,0};

  PID fx = PID(gains[0], integrals[0], 0, 0.002);
  PID fy = PID(gains[1], integrals[1], 0, 0.002);
  PID fz = PID(gains[2], integrals[2], 0, 0.002);
  fx.setInputLimits(-25.0, 25.0);
  fy.setInputLimits(-25.0, 25.0);
  fz.setInputLimits(-25.0, 25.0);
  fx.setOutputLimits(-0.0005, 0.0005);
  fy.setOutputLimits(0.0004, 0.0005);
  //fy.setOutputLimits(-0.0005, 0.0005);
  fz.setOutputLimits(-0.0005, 0.0005);

  // get loads in path
  vector<load> loads;
  vector<vector<double>> positions;
  log.GetLoadsFromCSV(follow_loads_path, loads);
  log.GetPositionsFromCSV(follow_loads_path, positions);

  // calibrate, transform, start load cell
  load_cell_->Initialize(Config::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION, Config::SAMPLE_RATE, Config::LOADCELL_CHANNEL.c_str());

  // start robot in cartesian impedance control mode
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    load_cell_->Stop();
    return err_val;
  }
  // set starting load to find
  load find_load = loads[path_point];
  fx.setSetPoint(loads[path_point].x);
  fy.setSetPoint(loads[path_point].y);
  fz.setSetPoint(loads[path_point].z);
  vector<double> find_pos = positions[0];

  // get starting process value
  load_cell_->GetLoads(load_cell_loads);
  fx.setProcessValue(load_cell_loads[0]);
  fy.setProcessValue(load_cell_loads[1]);
  fz.setProcessValue(load_cell_loads[2]);

  // debug log
  string dbg_base = "C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\logged\\";
  string dbg_name = dbg_base + output_position_filename +"_"+ std::to_string(path_point) + CSV;
  ofstream os(dbg_name);

  //fri_->GetMeasuredCartPose(pose_cmd);  // to fix rotations

  unsigned int timeout_counter = 0;
  DBGPRINT("Starting Nav By Bending Forces");
  while (path_point < loads.size()) {
    while (fx.getAbsDiff() > boundary_threshold[0] ||
      fy.getAbsDiff() > boundary_threshold[1] ||
      fz.getAbsDiff() > boundary_threshold[2]) {
      fri_->WaitForKRCTick();

      // FRI connection check
      if (!fri_->IsMachineOK()) {
        printf("ERROR, the machine is not ready anymore\n");
        DBGPRINT("KILLED AT: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", load_cell_loads[0], load_cell_loads[1], load_cell_loads[2], load_cell_loads[3], load_cell_loads[4], load_cell_loads[5]);
        for (auto pose : found_points_pose) {
          printf("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
        }
        load_cell_->Stop();
        os.close();
        return ERROR_MACHINE_NOT_OKAY;
      }

      printf("%d ", timeout_counter);

      // set FRI force term to self
      fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
      fri_->SetCommandedCartForcesAndTorques(est_ft);

      // timeout if in sink
      if (timeout_counter++ >= TIMEOUT_COUNT) {
        load_cell_->Stop();
        printf("Sink!\n");
        return Errors::ERROR_TIMEOUT_COUNT;
      }

      // get process value
      load_cell_->GetLoads(load_cell_loads);
      fx.setProcessValue(load_cell_loads[0]);
      fy.setProcessValue(load_cell_loads[1]);
      fz.setProcessValue(load_cell_loads[2]);

      // log pose
      fri_->GetMeasuredCartPose(pose_msr);
      LogPose(os, pose_msr, PrettyPrint::ENDLINE::ENDCOMMA);
      //printf("msr\n");
      //PrettyPrint::PrintPose(pose_msr);

      // max bounds timeout check
      if (!InBounds(find_pos, pose_msr, max_bounds)) {
        load_cell_->Stop();
        printf("Out of bounds!\n");
        return Errors::ERROR_OUT_OF_BOUNDS;  // out of max bounds error
      }

      computed[0] = fx.compute();
      computed[1] = fy.computeNonCenter();
      computed[2] = fz.compute();

      os << computed[0] << "," << computed[1] << "," << computed[2] << ",";
      printf("fx: %.7f, fy: %.7f, fz: %.7f\n", fx.prev_error_, fy.prev_error_, fz.prev_error_);
      
      if (abs(fx.prev_error_) > 0.05 || abs(fy.prev_error_) > 0.05 || abs(fz.prev_error_) > 0.05) {
        printf("currently poked\n");
        continue;
      }
      
      if (computed[1] <= 0.0004) {
        break;
      }
      //ForceMovement(pose_msr, computed, pose_cmd);

      fri_->SetCommandedCartPose(pose_cmd);
      //printf("cmd\n");
      //PrettyPrint::PrintPose(pose_cmd);
      LogPose(os, pose_cmd, PrettyPrint::ENDLINE::NEWLINE);
    }
    // found point
    DBGPRINT("DBG Point %d. @: %.2f, %.2f, %.2f\n", path_point, pose_msr[3] * 1000, pose_msr[7] * 1000, pose_msr[11] * 1000);
    find_load = loads[path_point];
    fx.setSetPoint(loads[path_point].x);
    fy.setSetPoint(loads[path_point].y);
    fz.setSetPoint(loads[path_point].z);

    find_pos = positions[path_point];
    path_point++;
    timeout_counter = 0;

    vector<float> v;
    for (unsigned int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
      v.push_back(pose_msr[i]);
    }
    found_points_pose.push_back(v);

    /*
    for (unsigned int i = 0; i < 250; ++i) {
      fri_->WaitForKRCTick();
      fri_->GetMeasuredCartPose(pose_msr);
      LogPose(os, pose_msr, PrettyPrint::ENDLINE::ENDCOMMA);
      os << "*" << "\n";
    }
    */

    // new dbg file
    dbg_name = dbg_base + output_position_filename + "_" + std::to_string(path_point) + CSV;
    os.close();
    os.open(dbg_name);
  }
  // wrap up
  load_cell_->Stop();
  for (auto pose : found_points_pose) {
    printf("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
  }
  Stats::GenerateNavByBendingPositionResults(found_points_pose, positions,
    dbg_base + output_position_filename + "_results" + CSV,
    avgs, stdevs);
  return SUCCESS;
}

int LWR::NavByBendingForces(string follow_loads_path, string output_position_filename) {
  Logging::Logger log = Logging::Logger();

  load current_load;
  unsigned int path_point = 0;
  int delay_time = 2000;  // ms delay between finding points
  int err_val = 0;
  load boundary_threshold = { 0, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f };
  vector<vector<float>> found_points_pose;
  
  float scale_x[NUMBER_OF_SCALING_FACTORS] = { -0.2f, -0.2f, -8.0f, -0.00000000001f, -0.00000000001f, -0.00000000001f };
  float scale_y[NUMBER_OF_SCALING_FACTORS] = { -1.0f, -0.8f, -0.05f, -0.00000000001f, -0.00000000001f, -0.00000000001f };
  float scale_z[NUMBER_OF_SCALING_FACTORS] = { -1.0f, -0.4f, -2.0f, -0.00000000001f, -0.00000000001f, -0.00000000001f };

  float* scale_ptr = scale_x;

  // get loads in path
  vector<load> loads;
  vector<vector<double>> positions;
  log.GetLoadsFromCSV(follow_loads_path, loads);
  log.GetPositionsFromCSV(follow_loads_path, positions);

  // calibrate, transform, start load cell
  load_cell_->Initialize(Config::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION, Config::SAMPLE_RATE, Config::LOADCELL_CHANNEL.c_str());

  // start robot in cartesian impedance control mode
  float fri_starting_loads[6] = { 0, 0, 0, 0, 0, 0 };
  //load_cell_->GetLoads(load_cell_loads);
  //SetKUKAFRILoads(load_cell_loads);
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    load_cell_->Stop();
    return err_val;
  }

  // set starting load to find
  load find_load = loads[path_point];
  
  // debug log
  string dbg_base = "C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\logged\\";
  string dbg_ext = ".csv";
  string dbg_name = dbg_base + output_position_filename +std::to_string(path_point) + dbg_ext;
  ofstream os(dbg_name);
  LogLoad(os, find_load);
  log.LogHeader(os);

  DBGPRINT("Starting Nav By Bending Forces");
  while (path_point < loads.size()) {
    fri_->WaitForKRCTick();

    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      DBGPRINT("Loads: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", load_cell_loads[0], load_cell_loads[1], load_cell_loads[2], load_cell_loads[3], load_cell_loads[4], load_cell_loads[5]);
      DBGPRINT("Position: %.3f, %.3f, %.3f", pose_msr[3], pose_msr[7], pose_msr[11]);
      for (auto pose : found_points_pose) {
        printf("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
      }
      load_cell_->Stop();
      return ERROR_MACHINE_NOT_OKAY;
    }

    // get current loadcell forces
    load_cell_->GetLoads(load_cell_loads);
    LogLoad(os, load_cell_loads, PrettyPrint::ENDCOMMA);

    // set FRI force term
    // SetKUKAFRILoads(load_cell_loads, &os);
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

   // subtract desired point from lc forces (shift number line)
    Utils::SetLoad(load_cell_loads, current_load);
    Utils::SetDesiredPosition(load_cell_loads, find_load);
    LogLoad(os, load_cell_loads, PrettyPrint::ENDCOMMA);
    
    // get current pose
    fri_->GetMeasuredCartPose(pose_msr);
    LogPose(os, pose_msr, PrettyPrint::ENDCOMMA);

    // in loads position
    if (Utils::ReachedAcceptableLoadPosition(current_load, find_load, boundary_threshold)) {
      DBGPRINT("Point %d found @: %.2f, %.2f, %.2f\n", path_point, pose_msr[3] * 1000, pose_msr[7] * 1000, pose_msr[11] * 1000);
      find_load = loads[++path_point];
      
      vector<float> v;
      for (unsigned int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
        v.push_back(pose_msr[i]);
      }
      found_points_pose.push_back(v);

      // new dbg file
      dbg_name = dbg_base + std::to_string(path_point) + dbg_ext;
      os.close();
      os.open(dbg_name);
      LogLoad(os, find_load);
      log.LogHeader(os);
      //delay(delay_time);

      /*
      if (path_point == 6) {
        scale_ptr = scale_y;
      }
      if (path_point == 11) {
        scale_ptr = scale_z;
      }
      */
    }

    // move to loads position
    else { 
      // calculate how much to move based on forces to reach desired position
      ForceMovement(pose_msr, scale_ptr, load_cell_loads, pose_cmd);
      fri_->SetCommandedCartPose(pose_cmd);
      LogPose(os, pose_cmd);
    }
  }

  load_cell_->Stop();
  for (auto pose : found_points_pose) {
    printf("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
  }
  //Stats::GenerateNavByBendingPositionResults(found_points_pose, positions,
  //  "C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\experiments\\prelim\\stats\\position_results.csv");
  return SUCCESS;
}

// Non-optical tracking movement compensation. 
int LWR::NavByWire() {

  float scale[6] = { 0.0005f, 0.0005f, 0.0005f, 1.0f, 1.0f, 1.0f };

  // Lower threshold for forces from the loadcell to avoid jitter
  float lc_lower_threshold[6] = { 0.5f, 0.5f, 0.5f, 0.02f, 0.02f, 0.02f };
  load_cell_->SetLowerForceThreshold(lc_lower_threshold);

  float load_cell_forces[6] = { 0, 0, 0, 0, 0, 0 };
  int err_val = 0;

  int run_time;
  int cycles;

  // calibrate, transform, start load cell
  load_cell_->Initialize(Config::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION, Config::SAMPLE_RATE, Config::LOADCELL_CHANNEL.c_str());

  // Let user move the robot into an awesome position
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_LOW, CARTESIAN_DAMPING_LOW, CARTESIAN_TORQUE_NONE);
  if (err_val != EOK) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    return err_val;
  }
  printf("8 SECONDS TO POSITION THE ROBOT...hurry\n");
  run_time = 8;
  cycles = 0;
  while (static_cast<float>(cycles)* fri_->GetFRICycleTime() < run_time) {
    fri_->GetMeasuredCartPose(pose_msr);
    fri_->SetCommandedCartPose(pose_msr);
    fri_->WaitForKRCTick();
    cycles++;
  }

  // Restore rigidity
  fri_->SetCommandedCartStiffness(CARTESIAN_STIFFNESS_HIGH);
  fri_->WaitForKRCTick();

  printf("BIASING load cell\n");
  delay(1000);
  load_cell_->SetBias();

  // start main control loop
  printf("NAV MODE ACTIVATED... for 180 seconds\n");
  int movement_time = 10;
  run_time = 180;
  cycles = 0;
  while (static_cast<float>(cycles) * fri_->GetFRICycleTime() < run_time) {
    fri_->WaitForKRCTick();
    if (!fri_->IsMachineOK()) {
       printf("ERROR, the machine is not ready anymore\n");
       return Errors::ERROR_MACHINE_NOT_OKAY;
     }
    // get current loadcell forces
    load_cell_->GetLoads(load_cell_forces);

    // get current pose
    fri_->GetMeasuredCartPose(pose_msr);

    // move to desired cartesian position upon force input
    if (load_cell_->ExceedsLowerForceThreshold()) {
      ForceMovement(pose_msr, scale, load_cell_forces, pose_cmd);
      fri_->SetCommandedCartPose(pose_cmd);
    }
    cycles++;
  }
  // stop load cell
  load_cell_->Stop();
  printf("STOPPED load cell\n");
  return EOK;
}
