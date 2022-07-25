std::vector<vector<float>> ReverseVector(std::vector<vector<float>>v)
{
  std::reverse(v.begin(), v.end());
  return v;
}


// Prompt user for direction and distance to indefinitely travel (e.g. X 4)

// Call nav by bending function
// If exceeds safety boundaries then restart at start point
// 

int LWR::PositionPIDBatchTuning() {
  const string TEST_PATH_FILENAME =
    "";
  const string OUTPUT_FILENAME =
    "";
  unsigned int direction = 0;  // 0 is forward
  Logging::Logger log = Logging::Logger();
  log.SetInputFile(TEST_PATH_FILENAME);
  ofstream os(OUTPUT_FILENAME);
  vector<vector<double>> positions;
  log.GetPositionsFromCSV(TEST_PATH_FILENAME, positions);

  gains_combinatoric generator;
  // PID = -5000, -20, 0
  // PID = -4950, -20, 0
  // PID = -4900, -20, 0
  // ...
  // PID = 0, -20, 0
  // PID = -5000, -19, 0
  // ...
  generator.x = { -5000.0f, 0.0f, 50.0f }; // [-5000, -4950, -4900, -4850 ..., 0]
  generator.y = { -20.0f, 0.0f, 1.0f }; // [-20, -19, -18, ... 0]
  generator.z = { 0.0f, 0.0f, 0.0f }; // [0, 0 ..., 0]

  unsigned int gen_count = 0;
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  unsigned int err_val = SUCCESS;
  const unsigned int POINT_DELAY = 1000;  // ms
  vector<vector<float>> cart_path_forward;
  vector<vector<float>> cart_path_backwards;
  vector<pair<string, double>> gen_results;

  // calibrate, transform, start load cell
  load_cell_->Initialize(Config::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION,
    Config::SAMPLE_RATE,
    Config::LOADCELL_CHANNEL.c_str());

  // parse log file for path
  err_val = log.ParseMotion(NULL, &cart_path_forward);
  if (err_val != SUCCESS) {
    printf("ERROR, could not parse input file\n");
    return err_val;
  }
  cart_path_backwards = ReverseVector(cart_path_forward);

  // start robot in cartesian impedance control mode   
  fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = StartCartesianImpedanceControlMode(CARTESIAN_STIFFNESS_HIGH,
    CARTESIAN_DAMPING_LOW, est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
    load_cell_->Stop();
    return err_val;
  }

  unsigned int path_count = 0;
  unsigned int count = 0;
  for (vector<float> val; generator(val);) {
    printf("Gains %d/%d\n", ++gen_count, generator.size);
    DBGPRINT("Starting Path Follow");
    path_count = 0;
    count = 0;

    switch (direction) {
      case 0:  // forward
        MoveToCartesianPosePIDv2(cart_path_forward[0]);
        for (vector<float> cart_pose : cart_path_forward) {
          printf("Point %d / %d\n", count, cart_path_forward.size() - 1);
          printf("Moving to: %.5f, %.5f, %.5f\n", cart_pose[3], cart_pose[7], cart_pose[11]);
          MoveToCartesianPosePIDv2(cart_pose, &path_count);
          delay(POINT_DELAY);
          count++;
        }
        direction = 1;  // backwards
        break;
      case 1:  // reverse
        MoveToCartesianPosePIDv2(cart_path_backwards[0]);
        for (vector<float> cart_pose : cart_path_backwards) {
          printf("Point %d / %d\n", count, cart_path_backwards.size() - 1);
          printf("Moving to: %.5f, %.5f, %.5f\n", cart_pose[3], cart_pose[7], cart_pose[11]);
          MoveToCartesianPosePIDv2(cart_pose, &path_count);
          delay(POINT_DELAY);
          count++;
        }
        direction = 0;  // backwards
        break;
    }
    string result_name = to_string(val[0]) + "_" + to_string(val[1]) + "_" + to_string(val[2]);
    //pair<string, double> result(result_name, path_count);
    //gen_results.push_back(result);
    os << result_name << "," << path_count << "\n";
  }

  // wrap up
  MoveToCartesianPosePIDv2(cart_path_forward[0]);
  delay(POINT_DELAY);
  load_cell_->Stop();
  return EOK;
}