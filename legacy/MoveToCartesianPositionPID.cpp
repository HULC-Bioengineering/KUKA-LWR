  int MoveToCartesianPosePIDv3(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]);
  int MoveToCartesianPosePIDv3(std::vector<float> pose);
  int MoveToCartesianPosePIDv4(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]);
  int MoveToCartesianPosePIDv4(std::vector<float> pose);

    void ForceMovement2(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    const float* scale,
    const float(&loads)[LOAD_CELL_DOF],
    float* next_pose);

      void CreateTransformationFromLoadValues(const float* scale, const float(&load_cell_forces)[LOAD_CELL_DOF], Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& lc_matrix);

  void SetNextStep(const float& err, const float& step_max, const float& step_min, const float& allowable_pos_err,
    float& position);
  void SetRotation(const Eigen::Vector3d& dsr, const float& rot_max, const float& rot_min, const float& allowable_rot_err,
    Eigen::Vector3d& msr_rot, float(&msr)[12]);

  // Move the robot based on force feedback from the load cell.
  int NavByWire();

  // Batch testing for nav by bending forces gains values. Computes averages
  // stdev for each trail, and logs passed and failed gains values.
  // Parameters:
  //   follow_loads_path: loads path to follow.
  //   output_position_filename: output pose filename.
  //   max_bounds: max allowable positional deviation from pose path.
  //   gains: set of gains values to evaluate.
  //   avgs: sets to average positional deviation for the trial.
  //   stdevs: sets to average stdevs for the trial.
    int BoundingBoxNavByBendingForces(std::string follow_loads_path,
    std::string output_position_filename,
    const float* max_bounds,
    std::vector<float>& gains,
    Eigen::Vector3d& avgs,
    Eigen::Vector3d& stdevs);

  int BoundingBoxNavByBendingForcesv2(std::string follow_loads_path,
    std::string output_position_filename,
    const float* max_bounds,
    std::vector<float>& gains,
    std::vector<float>& integrals,
    Eigen::Vector3d& avgs,
    Eigen::Vector3d& stdevs);

  int NavByBendingForces(std::string follow_loads_path,
    std::string output_position_filename);
  
  // Move the LWR to the input cartesian pose. Uses an internal PID controller.
  // Parameters:
  //   pose: row major transformation matrix as vector
  int MoveToCartesianPose(const std::vector<float>& pose);

  // Move the LWR to the input cartesian pose. Uses an internal PID controller.
  // Parameters:
  //   pose: row major transformation matrix as array
  int MoveToCartesianPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]);

    /// DH parameters of the robot
  static const double D1;
  static const double D3;
  static const double D5;
  static const double D7;
  static const double ALPHA;
  
  // rotation limits of each joint (rad)
  static const double JNT_LIMITS [];

  static bool ikSolver(std::vector<double>& jntPosMsr, double(&pose)[12],
    std::vector<double>& jntPosDsr);

  static bool fkSolver(const std::vector<double> & jntPosDsr,
    double(&dsr_pose)[12]);

  static void DH(double a, double alpha, double d, double theta,
    Eigen::Matrix4d& FXX);

  // Find T base, ti+1
  // base     ti       base
  //     T      T  =       T
  //   ti   ti+1       ti+1
  // Parameters:
  //   pose: current pose
  //   scale: 
  //   loads: current load cell loads transformed to tool frame
  //   next_pose: transformation matrix which should be sent to motion kernal
  void ForceMovement(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    const float* scale,
    const float(&loads)[LOAD_CELL_DOF],
    float* next_pose);

  // Creates the load cell transformation matrix by using the
  // X-Y-Z Extrinsic Tait Bryant rotation sequence.
  // Parameters:
  //   scale: 
  //   loads: current load cell loads transformed to tool frame
  //   lc_matrix: resulting transformation matrix
  void CreateTransformationFromLoadValues(const float* scale,
    const float(&loads)[LOAD_CELL_DOF],
    boost::numeric::ublas::matrix<float>* lc_matrix);


int LWR::MoveToCartesianPosePIDv3(vector<float> pose) {
  float arr_position[NUMBER_OF_FRAME_ELEMENTS];
  std::copy(pose.begin(), pose.end(), arr_position);
  return MoveToCartesianPosePIDv3(arr_position);
}

int LWR::MoveToCartesianPosePIDv3(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
  ofstream of("C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\logs\\pid.csv");
  const float ERR_TOLERANCE[6] = { 0.00005, 0.00005, 0.00005, 0.002, 0.002, 0.002 };  // mm, rad
  int err_val = SUCCESS;
  int SLACK_VAR = 0.00005; // mm

  PID6DOF pid = PID6DOF();
  pid.setSetPoint(pose);

  // start robot in cartesian impedance control mode
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    return err_val;
  }
  fri_->GetMeasuredCartPose(pose_msr);
  pid.setProcessValue(pose_msr);
  

  while (!InThreshold(ERR_TOLERANCE, SLACK_VAR, abs(pose[3] - pose_msr[3]), abs(pose[7] - pose_msr[7]), abs(pose[11] - pose_msr[11])))
  {
    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return ERROR_MACHINE_NOT_OKAY;
    }

    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    // set new process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid.setProcessValue(pose_msr);

    // compute and add to measured pose
    pid.compute(pose_msr);

    // send pose to KRC
    fri_->SetCommandedCartPose(pose_msr);
    fri_->WaitForKRCTick();

    // set new process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid.setProcessValue(pose_msr);
  }

  DBGPRINT("Moved to Position: %.5f, %.5f, %.5f", pose_msr[3], pose_msr[7], pose_msr[11]);
  return SUCCESS;
}

int LWR::MoveToCartesianPosePIDv4(vector<float> pose) {
  float arr_position[NUMBER_OF_FRAME_ELEMENTS];
  std::copy(pose.begin(), pose.end(), arr_position);
  return MoveToCartesianPosePIDv3(arr_position);
}
// first translations then rotations using 2x 3DOF PID controllers
int LWR::MoveToCartesianPosePIDv4(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
  const float ERR_TOLERANCE[6] = { 0.00005, 0.00005, 0.00005, 0.0001, 0.0001, 0.0001 };  // mm, rad
  int err_val = SUCCESS;
  int SLACK_TRANS = 0.00005; // mm
  int SLACK_ROT = 0.002; // rad

  PID6DOF pid_translations = PID6DOF();
  pid_translations.setSetPoint(pose);
  pid_translations.setGains({1200, 1200, 1200, 0, 0, 0});
  pid_translations.setIntegrals({ 2, 2, 2, 0, 0, 0 });

  PID6DOF pid_rotations = PID6DOF();
  pid_rotations.setSetPoint(pose);
  pid_rotations.setGains({ 0, 0, 0, 10, 10, 10 });
  pid_rotations.setIntegrals({0, 0, 0, 5, 5, 5});

  // start robot in cartesian impedance control mode
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    return err_val;
  }
  fri_->GetMeasuredCartPose(pose_msr);
  pid_translations.setProcessValue(pose_msr);
  
  // translations
  while (!InThreshold(ERR_TOLERANCE, SLACK_TRANS, abs(pose[3] - pose_msr[3]), abs(pose[7] - pose_msr[7]), abs(pose[11] - pose_msr[11])))
  {
    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return ERROR_MACHINE_NOT_OKAY;
    }

    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    // set new process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid_translations.setProcessValue(pose_msr);

    // compute and add to measured pose
    pid_translations.compute(pose_msr);

    // send pose to KRC
    fri_->SetCommandedCartPose(pose_msr);
    fri_->WaitForKRCTick();

    // set new process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid_translations.setProcessValue(pose_msr);
  }

  // rotations
  pid_rotations.setProcessValue(pose_msr);
  while (1)
 // while (!InThreshold(ERR_TOLERANCE, SLACK_ROT,
 //   abs(pid_rotations.a_.getSetPoint() - pid_rotations.a_.getProcessValue()),
 //   abs(pid_rotations.b_.getSetPoint() - pid_rotations.b_.getProcessValue()),
 //   abs(pid_rotations.c_.getSetPoint() - pid_rotations.c_.getProcessValue())))
  // while (!InThreshold(ERR_TOLERANCE, SLACK_ROT, abs(pose[3] - pose_msr[3]), abs(pose[7] - pose_msr[7]), abs(pose[11] - pose_msr[11])))
  {
    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore 2\n");
      return ERROR_MACHINE_NOT_OKAY;
    }
    //printf("a: %.5f\n", abs(pid_rotations.a_.getSetPoint() - pid_rotations.a_.getProcessValue()));
    //printf("b: %.5f\n", abs(pid_rotations.b_.getSetPoint() - pid_rotations.b_.getProcessValue()));
    //printf("c: %.5f\n", abs(pid_rotations.c_.getSetPoint() - pid_rotations.c_.getProcessValue()));
    
    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    // set new process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid_rotations.setProcessValue(pose_msr);

    // compute and add to measured pose
    pid_rotations.compute(pose_msr);

    // send pose to KRC
    fri_->SetCommandedCartPose(pose_msr);
    fri_->WaitForKRCTick();

    // set new process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid_rotations.setProcessValue(pose_msr);
    PrettyPrint::PrintPose(pose);
  }

  DBGPRINT("Moved to Position: %.5f, %.5f, %.5f", pose_msr[3], pose_msr[7], pose_msr[11]);
  return SUCCESS;
}

// Move cartesian pose along line between current point and desired point.
// Super ghetto as it doesn't account for velocities, accelerations, or path
// planning that isn't linear. Mostly intended for simple 2D motion.
int LWR::MoveToCartesianPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
  int err_val = SUCCESS;
  float ALLOWABLE_POS_ERR = 0.0003;
  float STEP_MAX = 0.0005f;
  float STEP_MIN = 0.00005f;
  float ROT_MAX = 0.001;
  float ROT_MIN = 0.0001;
  float ALLOWABLE_ROT_ERR = 0.1;

  float err_x = 0;
  float err_y = 0;
  float err_z = 0;

  // start robot in cartesian impedance control mode
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    return err_val;
  }

  // get desired rotation
  Vector3d dsr_rot;
  BoostMatrix::GetXYZEulerRotationsFromPose(pose, dsr_rot);
  // fixed z,x,y

  // Calculate difference between measured and desired poses.
  fri_->GetMeasuredCartPose(pose_msr);
  err_x = pose[3] - pose_msr[3];
  err_y = pose[7] - pose_msr[7];
  err_z = pose[11] - pose_msr[11];
  Vector3d msr_rot;
  BoostMatrix::GetXYZEulerRotationsFromPose(pose_msr, msr_rot);
  Vector3d err_rot = dsr_rot - msr_rot;

  // STEP 1: translate
  while (abs(err_x) > ALLOWABLE_POS_ERR ||
    abs(err_y) > ALLOWABLE_POS_ERR ||
    abs(err_z) > ALLOWABLE_POS_ERR) {
    
    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return ERROR_MACHINE_NOT_OKAY;
    }

    // set next pose position values
    SetNextStep(err_x, STEP_MAX, STEP_MIN, ALLOWABLE_POS_ERR, pose_msr[3]);
    SetNextStep(err_y, STEP_MAX, STEP_MIN, ALLOWABLE_POS_ERR, pose_msr[7]);
    SetNextStep(err_z, STEP_MAX, STEP_MIN, ALLOWABLE_POS_ERR, pose_msr[11]);
    //printf("\n");
    // send pose to KRC and feed loads into self
    fri_->SetCommandedCartPose(pose_msr);
    fri_->WaitForKRCTick();

    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    // set new err values
    fri_->GetMeasuredCartPose(pose_msr);
    pose[3] - pose_msr[3] == err_x && err_x > STEP_MIN && err_x < STEP_MAX ? err_x *= 1.2 : err_x = pose[3] - pose_msr[3];
    pose[7] - pose_msr[7] == err_y && err_y > STEP_MIN && err_y < STEP_MAX ? err_y *= 1.2 : err_y = pose[7] - pose_msr[7];
    pose[11] - pose_msr[11] == err_z && err_z > STEP_MIN && err_z < STEP_MAX ? err_z *= 1.2 : err_z = pose[11] - pose_msr[11];
  }

  // Step 2: Rotate
  /*
  while (abs(err_rot[0]) > ALLOWABLE_ROT_ERR ||
    abs(err_rot[1]) > ALLOWABLE_ROT_ERR ||
    abs(err_rot[2]) > ALLOWABLE_ROT_ERR) {

    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return ERROR_MACHINE_NOT_OKAY;
    }

    SetRotation(err_rot, ROT_MAX, ROT_MIN, ALLOWABLE_ROT_ERR, msr_rot, pose_msr);

    // send pose to KRC and feed loads into self
    fri_->SetCommandedCartPose(pose_msr);
    fri_->WaitForKRCTick();

    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    // set new err values
    fri_->GetMeasuredCartPose(pose_msr);
    BoostMatrix::GetXYZEulerRotationsFromPose(pose_msr, msr_rot);
    err_rot = dsr_rot - msr_rot;
    printf("%.5f, %.5f, %.5f\n", err_rot[0], err_rot[1], err_rot[2]);
  }
  */
  return SUCCESS;
}

// fixed z,x,y
void LWR::SetRotation(const Vector3d& err, const float& rot_max, const float& rot_min, const float& allowable_rot_err,
  Vector3d& msr_rot, float (&msr)[12]) {

  for (unsigned int i = 0; i < 3; ++i) {
    if (abs(err[i]) > allowable_rot_err) {
      if (abs(err[i]) > rot_max) {
        if (err[i] < 0) {
          msr_rot[i] -= rot_max;
        }
        else {
          msr_rot[i] += rot_max;
        }
      }
      else if (abs(err[i]) < rot_min) {
        ; // do nothing
      }
      else {
        msr_rot[i] += err[i];
      }
    }
  }

  Matrix3d n;
  n = Eigen::AngleAxisd(msr_rot[1], Vector3d::UnitY())
    *Eigen::AngleAxisd(msr_rot[0], Vector3d::UnitX())
    *Eigen::AngleAxisd(msr_rot[2], Vector3d::UnitZ());

  BoostMatrix::BuildPoseFromRotationMatrix(n, msr);
}

void LWR::SetNextStep(const float& err, const float& step_max, const float& step_min, const float& allowable_pos_err,
  float& position) {
  //printf("%.5f\t", position);
  if (abs(err) > allowable_pos_err) {
    if (abs(err) > step_max) {
      if (err < 0) {
        position -= step_max;
      }
      else {
        position += step_max;
      }
    }
    else if (abs(err) < step_min) {
      ; // do nothing
    }
    else {
      position += err;
    }
  }
  //printf("%.5f\t", position);
}

// set pose for rotation matrix
void SetMeasuredPIDPose(const float(&pose)[12], float pid_a, float pid_b, float pid_c) {
  /*
  Vector3d process_rot;
  BoostMatrix::GetXYZEulerRotationsFromPose(process_pose, process_rot);
  pid_a.setProcessValue(process_rot[0]);
  pid_b.setProcessValue(process_rot[1]);
  pid_c.setProcessValue(process_rot[2]);
  */
}


int LWR::MoveToCartesianPosePID(vector<float> pose) {
  float arr_position[NUMBER_OF_FRAME_ELEMENTS];
  std::copy(pose.begin(), pose.end(), arr_position);
  return MoveToCartesianPosePID(arr_position);
}

// Move cartesian pose along line between current point and desired point.
// Super ghetto as it doesn't account for velocities, accelerations, or path
// planning that isn't linear. Mostly intended for simple 2D motion.
int LWR::MoveToCartesianPosePID(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
  ofstream of("C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\logs\\pid.csv");
  const float ERR_TOLERANCE[6] = { 0.00005, 0.00005, 0.00005, 1, 1, 1 };
  int err_val = SUCCESS;
  int SLACK_VAR = 0.00005;  // 2mm

  // 6 axes for control by the PID - 3 translational, 3 rotational
  PID pid_x = PID(1200, 2, 0, 0.002);
  PID pid_y = PID(1200, 2, 0, 0.002);
  PID pid_z = PID(1200, 2, 0, 0.002);
  //PID pid_a = PID(100, 0, 0, 0.002);
  //PID pid_b = PID(100, 0, 0, 0.002);
  //PID pid_c = PID(100, 0, 0, 0.002);

  // start robot in cartesian impedance control mode
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start robot in Cartesian Impedance Control Mode\n");
    return err_val;
  }
  fri_->GetMeasuredCartPose(pose_msr);

  // set PID process pose
  pid_x.setProcessValue(pose_msr[3]);
  pid_y.setProcessValue(pose_msr[7]);
  pid_z.setProcessValue(pose_msr[11]);
  //SetPIDProcessValues(pose_msr, pid_a, pid_b, pid_c);

  // set PID setpoint
  pid_x.setSetPoint(pose[3]);
  pid_y.setSetPoint(pose[7]);
  pid_z.setSetPoint(pose[11]);
  //SetPIDSetPointValues(pose, pid_a, pid_b, pid_c);

  float time_interval = 0;
  while (!InThreshold(ERR_TOLERANCE, SLACK_VAR, abs(pose[3] - pose_msr[3]), abs(pose[7] - pose_msr[7]), abs(pose[11] - pose_msr[11])))
  // while (1) // infinite loop
  {
    // FRI connection check
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return ERROR_MACHINE_NOT_OKAY;
    }

    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    // set next position process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid_x.setProcessValue(pose_msr[3]);
    pid_y.setProcessValue(pose_msr[7]);
    pid_z.setProcessValue(pose_msr[11]);
    //SetPIDProcessValues(pose_msr, pid_a, pid_b, pid_c);
    //printf("%.6f\t%.6f\t%.6f\t",pid_x.compute(), pid_y.compute(), pid_z.compute());

    // compute and add to measured pose
    pose_msr[3] += pid_x.compute();
    pose_msr[7] += pid_y.compute();
    pose_msr[11] += pid_z.compute();
    //printf("\n");
    // #TODO rotation values to XYZ Extrinsic Tait-Bryan
    //SetMeasuredPIDPose(pose_msr, pid_a.compute(), pid_b.compute(), pid_c.compute());

    // send pose to KRC
    fri_->SetCommandedCartPose(pose_msr);
    //printf("%.3f\t%.3f\t%.3f\n", pose_msr[3], pose_msr[7], pose_msr[11]);
    fri_->WaitForKRCTick();
    time_interval++;

    // set new process values
    fri_->GetMeasuredCartPose(pose_msr);
    pid_x.setProcessValue(pose_msr[3]);
    pid_y.setProcessValue(pose_msr[7]);
    pid_z.setProcessValue(pose_msr[11]);
    
    //SetPIDProcessValues(pose_msr, pid_a, pid_b, pid_c);
    // printf("%.3f\t%.3f\t%.3f\n", pose_msr[3], pose_msr[7], pose_msr[11]);
  }

  float position_x = pose_msr[3];
  float position_y = pose_msr[7];
  float position_z = pose_msr[11];

  // rotations
  Eigen::Matrix3f start;
  start <<
    pose_msr[0], pose_msr[1], pose_msr[2],
    pose_msr[4], pose_msr[5], pose_msr[6],
    pose_msr[8], pose_msr[9], pose_msr[10];
  Eigen::Quaternionf qa(start);

  Eigen::Matrix3f end;
  end << pose[0], pose[1], pose[2],
    pose[4], pose[5], pose[6],
    pose[8], pose[9], pose[10];
  Eigen::Quaternionf qb(end);

  Eigen::Matrix3f cur;
  Eigen::Quaternionf qres;
  unsigned int steps = 1000;

  //unsigned int t = 0;
 // while (qa.angularDistance(qb) > 0.0174532925) {
  for (int t = 0; t < steps; ++t) {
    if (!fri_->IsMachineOK()) {
      printf("ERROR, the machine is not ready anymore\n");
      return ERROR_MACHINE_NOT_OKAY;
    }

    // set FRI force term to self
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    fri_->SetCommandedCartForcesAndTorques(est_ft);

    qres = qa.slerp(float(t) / steps, qb);
    qres.normalize();
    cur = qres.toRotationMatrix();
    pose_msr[0] = cur(0,0);
    pose_msr[1] = cur(0,1);
    pose_msr[2] = cur(0,2);
    pose_msr[3] = position_x;
    pose_msr[4] = cur(1,0);
    pose_msr[5] = cur(1,1);
    pose_msr[6] = cur(1,2);
    pose_msr[7] = position_y;
    pose_msr[8] = cur(2,0);
    pose_msr[9] = cur(2,1);
    pose_msr[10] = cur(2,2);
    pose_msr[11] = position_z;

    // send pose to KRC
    fri_->SetCommandedCartPose(pose_msr);
    fri_->WaitForKRCTick();
  }
  DBGPRINT("Moved to Position: %.5f, %.5f, %.5f", pose_msr[3], pose_msr[7], pose_msr[11]);
  return SUCCESS;
}

void SetPIDSetPointValues(const float(&setpoint_pose)[12], PID& pid_a, PID& pid_b, PID& pid_c) {
  Vector3d setpoint_rot;
  BoostMatrix::GetXYZEulerRotationsFromPose(setpoint_pose, setpoint_rot);
  pid_a.setSetPoint(setpoint_rot[0]);
  pid_b.setSetPoint(setpoint_rot[1]);
  pid_c.setSetPoint(setpoint_rot[2]);
}

void SetPIDProcessValues(const float(&process_pose)[12], PID& pid_a, PID& pid_b, PID& pid_c) {
  Vector3d process_rot;
  BoostMatrix::GetXYZEulerRotationsFromPose(process_pose, process_rot);
  pid_a.setProcessValue(process_rot[0]);
  pid_b.setProcessValue(process_rot[1]);
  pid_c.setProcessValue(process_rot[2]);
}