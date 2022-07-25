

    void GetXYZEulerRotationsFromPose(
      const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
      Eigen::Vector3d& rot);
    double Round(double value, int precision);
    bool IsWholeNumber(const std::string& s);
    void LogHeader(std::ofstream& os);
    int GetPositionsFromCSV(std::string positions_file_path,
      std::vector<std::vector<double>>& positions);
    int WritePositions(std::string output_filename,
      std::vector<std::vector<float>> poses);
    void LogSolidGains(std::ofstream& os, std::vector<float>& gains,
      Eigen::Vector3d& avgs, Eigen::Vector3d& stdev);
    void LogGains(std::ofstream& os, std::vector<float>& gains);
    int GetLoadsFromCSV(std::string forces_file_path,
      std::string moments_file_path, std::vector<load>& loads);
    int GetLoadsFromCSV(std::string forces_file_path, std::vector<float>& loads);
    int GetLoadsFromCSV(std::string loads_file_path, std::vector<std::vector<double>>& loads);
    int GetLoadsFromCSV(std::string loads_file_path, std::vector<load>& loads);
    int WriteLoads(std::string output_loads_filename,
      std::string path_filename, const float(&starting_joints)[7],
      const float(&starting_pose)[12], std::vector<load>& loads);
    int LogAvgStdevMaxdev(std::string output_file,
      std::vector<load>& avg, std::vector<load>& stdev,
      std::vector<load>& maxdev);
    void WriteLoadAndPosition(std::ofstream& os, load l,
      float(&pose)[NUMBER_OF_FRAME_ELEMENTS]);
    int WriteLoadsAndPositions(std::string output_file_name,
      std::vector<load>& loads, std::vector<load>& pos_rot);









  // #TODO fix me
  void MotionParser::GetXYZEulerRotationsFromPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], Eigen::Vector3d& rot) {
    Matrix3f m;
    m << pose[0], pose[1], pose[2],
      pose[4], pose[5], pose[6],
      pose[8], pose[9], pose[10];

    rot = m.eulerAngles(1, 0, 2); // y,x,z  // fixed z,x,y
  }
  bool MotionParser::IsWholeNumber(const std::string& s) {
    return !s.empty() && std::find_if(s.begin(),
      s.end(), [](char c) { return !isdigit(c); }) == s.end();
  }
  double MotionParser::Round(double value, int precision) {
    int adjustment = static_cast<int>(pow(10, precision));
    return floor(value*(adjustment)+0.5) / adjustment;
  }

  void MotionParser::LogHeader(std::ofstream& os) {
    os <<
      "Raw LC Fx, Raw LC Fy, Raw LC Fz, Raw LC Mx, Raw LC My, Raw LC Mz," <<
      "Dsr LC Fx, Dsr LC Fy, Dsr LC Fz, Dsr LC Mx, Dsr LC My, Dsr LC Mz," <<
      "Msr X, Msr Y, Msr Z," <<
      "Dsr X, Dsr Y, Dsr Z,\n";
  }

  // HEADER FORMAT / INFORMATION
  void MotionParser::LogSolidGains(ofstream& os, vector<float>& gains, Vector3d& avgs, Vector3d& stdev) {
    os << Round(gains[0], 2) << "," << Round(gains[1], 2) << "," << Round(gains[2], 2) << "," <<
      avgs[0] << "," << avgs[1] << "," << avgs[2] << "," <<
      stdev[0] << "," << stdev[1] << "," << stdev[2] << "\n" << std::flush;
  }

  void MotionParser::LogGains(std::ofstream& os, std::vector<float>& gains) {
    os << gains[0] << "," << gains[1] << "," << gains[2] << "\n" << std::flush;
  }

  int MotionParser::LogAvgStdevMaxdev(string output_file, vector<load>& avg, vector<load>& stdev, vector<load>& maxdev) {

    std::ofstream output_stream;
    output_stream.open(output_file);
    output_stream << "Header Size:\t" << 4 << "\n";
    output_stream << "Name:\tStats for Nerds\n";
    output_stream << "Column Fields:\t" << "Item\t" << "Forces\t" << "Moments" << "\n";
    output_stream << "Column Element Size:\t" << "1\t" << "3\t" << "3" << "\n";
    output_stream << "Stat:\t" << "Mean" << "\n";
    for (vector<load>::iterator it = avg.begin(); it != avg.end(); ++it) {
      output_stream << it->t << "\t" << it->x << "\t" << it->y << "\t" << it->z << "\t" << it->mx << "\t" << it->my << "\t" << it->mz << "\n";
    }
    output_stream << "Stat:\t" << "Standard Deviation" << "\n";
    for (vector<load>::iterator it = stdev.begin(); it != stdev.end(); ++it) {
      output_stream << it->t << "\t" << it->x << "\t" << it->y << "\t" << it->z << "\t" << it->mx << "\t" << it->my << "\t" << it->mz << "\n";
    }
    output_stream << "Stat:\t" << "Max Deviation" << "\n";
    for (vector<load>::iterator it = maxdev.begin(); it != maxdev.end(); ++it) {
      output_stream << it->t << "\t" << it->x << "\t" << it->y << "\t" << it->z << "\t" << it->mx << "\t" << it->my << "\t" << it->mz << "\n";
    }
    output_stream.close();

    return SUCCESS;
  }

  int MotionParser::GetPositionsFromCSV(string positions_file_path, vector<vector<double>>& positions) {
    std::ifstream loads_input_file(positions_file_path);
    if (!loads_input_file.is_open()) {
      printf("Unable to open forces input file");
      return Errors::ERROR_INVALID_INPUT_FILE;
    }

    string line;
    vector<string> line_split_loads;
    while (std::getline(loads_input_file, line)) {
      boost::algorithm::split(line_split_loads, line, boost::is_any_of(","),
        boost::token_compress_on);
      if (line_split_loads.size() == 1) {
        break;
      }

      vector<double> v;
      v.push_back(stod(line_split_loads[7]));
      v.push_back(stod(line_split_loads[8]));
      v.push_back(stod(line_split_loads[9]));
      positions.push_back(v);
    }
    return SUCCESS;
  }

  // Get Fx, Fy, Fz, Mx, My, Mz from logged force file
  int MotionParser::GetLoadsFromCSV(string loads_file_path, vector<vector<double>>& loads)  {
    std::ifstream loads_input_file(loads_file_path);
    if (!loads_input_file.is_open()) {
      printf("Unable to open forces input file");
      return Errors::ERROR_INVALID_INPUT_FILE;
    }

    string line;
    vector<string> line_split_loads;
    while (std::getline(loads_input_file, line)) {
      boost::algorithm::split(line_split_loads, line, boost::is_any_of(","),
        boost::token_compress_on);

      if (line_split_loads.size() == 1) {
        break;
      }
      loads.push_back({
        stof(line_split_loads[1]),  // fx
        stof(line_split_loads[2]),  // fy
        stof(line_split_loads[3]),  // fz
        stof(line_split_loads[4]),  // mx
        stof(line_split_loads[5]),  // my
        stof(line_split_loads[6]),  // mz
      });
    }
    return SUCCESS;
  }

  int MotionParser::GetLoadsFromCSV(string loads_file_path, vector<load>& loads) {
    // ensure empty loads as starting point
    //loads.clear();

    string line;

    std::ifstream loads_input_file(loads_file_path);
    if (!loads_input_file.is_open()) {
      printf("Unable to open forces input file");
      return Errors::ERROR_INVALID_INPUT_FILE;
    }

    /*
    // ignore header information
    for (int i = 0; i < 5; ++i) {
    std::getline(loads_input_file, line);
    }
    */

    // read force file reaction force lines
    vector<string> line_split_loads;

    while (1) {
      std::getline(loads_input_file, line);
      boost::algorithm::split(line_split_loads, line, boost::is_any_of(","),
        boost::token_compress_on);

      if (line_split_loads.size() == 1) {
        break;
      }
      loads.push_back({
        stoi(line_split_loads[0]),  // t
        stof(line_split_loads[1]),  // fx
        stof(line_split_loads[2]),  // fy
        stof(line_split_loads[3]),  // fz
        stof(line_split_loads[4]),  // mx
        stof(line_split_loads[5]),  // my
        stof(line_split_loads[6]),  // mz
      });
    }
    return SUCCESS;
  }

  int MotionParser::GetLoadsFromCSV(string forces_file_path, string moments_file_path, vector<load>& loads) {

    // ensure empty loads as starting point
    loads.clear();

    string line;

    // open file(s)
    std::ifstream forces_input_file(forces_file_path);
    std::ifstream moments_input_file(moments_file_path);

    if (!forces_input_file.is_open()) {
      printf("Unable to open forces input file");
      return Errors::ERROR_INVALID_INPUT_FILE;
    }

    if (!moments_input_file.is_open()) {
      printf("Unable to open moments input file");
      return Errors::ERROR_INVALID_INPUT_FILE;
    }

    // ignore header information
    for (int i = 0; i < 10; ++i) {
      std::getline(forces_input_file, line);
      std::getline(moments_input_file, line);
    }

    // add point 0 to forces and moments
    loads.push_back({ 0, 0, 0, 0, 0, 0, 0 });

    // read force file reaction force lines
    vector<string> line_split_forces;
    vector<string> line_split_moments;

    while (1) {
      std::getline(forces_input_file, line);
      boost::algorithm::split(line_split_forces, line, boost::is_any_of(","),
        boost::token_compress_on);
      std::getline(moments_input_file, line);
      boost::algorithm::split(line_split_moments, line, boost::is_any_of(","),
        boost::token_compress_on);

      if (line_split_moments.size() == 1) {
        break;
      }


      // only get values at int time: 1,2,3,4... Ignore intermediary values
      boost::algorithm::trim(line_split_forces[1]);
      if (IsWholeNumber(line_split_forces[1])) {
        loads.push_back({
          stoi(line_split_forces[1]),  // t
          stof(line_split_forces[2]),  // x
          stof(line_split_forces[3]),  // y
          stof(line_split_forces[4]),  // z
          stof(line_split_moments[2]),  // mx
          stof(line_split_moments[3]),  // my
          stof(line_split_moments[4]),  // mz
        });
      }
    }
    return SUCCESS;
  }

  void MotionParser::WriteLoadAndPosition(std::ofstream& os, load l, float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
    Vector3d rot;

    GetXYZEulerRotationsFromPose(pose, rot);
    os << l.t << "," <<
      l.x << "," << l.y << "," << l.z << "," <<
      l.mx << "," << l.my << "," << l.mz << "," <<
      pose[3] << "," << pose[7] << "," << pose[11] << "," <<
      rot[0] << "," << rot[1] << "," << rot[2] << "\n";
  }
  //Header Size : 4
  //Path Name : prelim
  //Column Fields : Item	JointPosition	CartesianPose
  //Column Element Size : 1	7	12
  //Entries : 5
  //0	10	10	10	10	10	10	10	1	0	0 - 0.500	0	1	0	0.04	0	0	1	0.06
  int MotionParser::WriteLoads(std::string output_loads_filename, std::string path_filename, const float(&starting_joints)[7], const float(&starting_pose)[12], std::vector<load>& loads) {
    std::ofstream output_stream;
    output_stream.open(output_loads_filename);
    output_stream << "Header Size:\t" << 4 << "\n";
    output_stream << "Path Name:\t" << path_filename << "\n";
    output_stream << "Column Fields:\t" << "Item\t" << "Forces\t" << "Moments" << "\n";
    output_stream << "Column Element Size:\t" << "1\t" << "3\t" << "3" << "\n";
    output_stream << "Entries:\t" << loads.size() << "\n";
    for (vector<load>::iterator it = loads.begin(); it != loads.end(); ++it) {
      output_stream << it->t << "\t" << it->x << "\t" << it->y << "\t" << it->z << "\t" << it->mx << "\t" << it->my << "\t" << it->mz << "\n";
    }
    output_stream.close();
    return SUCCESS;
  }

  // assumes same vector size
  /*
  int MotionParser::WriteLoadsAndPositions(std::string output_filename, std::vector<position>& positions, std::vector<load>& loads) {
    std::ofstream output_stream(output_filename);

    for (unsigned int i = 0; i < loads.size(); ++i) {
      output_stream << loads[i].t << "," << loads[i].x << "," << loads[i].y << "," << loads[i].z << "," << loads[i].mx << "," << loads[i].my << "," << loads[i].mz << "," << positions[i].x << "," << positions[i].y << "," << positions[i].z << "\n";
    }
    return SUCCESS;
  }
  */

  int MotionParser::WriteLoadsAndPositions(std::vector<load>& loads, std::vector<load>& pos_rot) {
    if (output_filename_ != "") {
      WriteLoadsAndPositions(output_filename_, loads, pos_rot);
    }
    else {
      printf("Output file not set");
      return 1;
    }
    return SUCCESS;
  }

  int MotionParser::WriteLoadsAndPositionsStream(std::vector<load>& loads, std::vector<load>& pos_rot) {
    for (unsigned int i = 0; i < loads.size(); ++i) {
      of_stream_ << loads[i].t << "," <<
        loads[i].x << "," << loads[i].y << "," << loads[i].z << "," <<
        loads[i].mx << "," << loads[i].my << "," << loads[i].mz << "," <<
        pos_rot[i].x << "," << pos_rot[i].y << "," << pos_rot[i].z << "," <<
        pos_rot[i].mx << "," << pos_rot[i].my << "," << pos_rot[i].mz << "\n";
    }
    return SUCCESS;
  }
  int MotionParser::WriteLoadsAndPositions(std::string output_filename, std::vector<load>& loads, std::vector<load>& pos_rot) {
    std::ofstream output_stream(output_filename);

    for (unsigned int i = 0; i < loads.size(); ++i) {
      output_stream << loads[i].t << "," <<
        loads[i].x << "," << loads[i].y << "," << loads[i].z << "," <<
        loads[i].mx << "," << loads[i].my << "," << loads[i].mz << "," <<
        pos_rot[i].x << "," << pos_rot[i].y << "," << pos_rot[i].z << "," <<
        pos_rot[i].mx << "," << pos_rot[i].my << "," << pos_rot[i].mz << "\n";
    }
    return SUCCESS;
  }



  /*
  int MotionParser::WritePositions(string output_filename, vector<position> positions) {
    std::ofstream output_stream;
    output_stream.open(output_filename);
    output_stream << "Header Size:\t" << 3 << "\n";
    output_stream << "Column Fields:\t" << "Item\t" << "Position" << "\n";
    output_stream << "Column Element Size:\t" << "1\t" << "3\t" << "\n";
    for (vector<position>::iterator it = positions.begin(); it != positions.end(); ++it) {
      output_stream << it->t << "\t" << it->x << "\t" << it->y << "\t" << it->z << "\n";
    }
    output_stream.close();
    return SUCCESS;
  }
  */

  int MotionParser::WritePositions(string output_filename, vector<vector<float>> poses) {
    std::ofstream output_stream;
    output_stream.open(output_filename);
    output_stream << "Header Size:\t" << 3 << "\n";
    output_stream << "Column Fields:\t" << "Item\t" << "Position" << "\n";
    output_stream << "Column Element Size:\t" << "1\t" << "3\t" << "\n";
    int count = 0;
    for (auto pose : poses) {
      output_stream << count++ << pose[3] << "\t" << pose[7] << "\t" << pose[11] << "\n";
    }
    output_stream.close();
    return SUCCESS;
  }