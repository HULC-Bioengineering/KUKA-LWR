// Copyright 2014 Matthew Stokes. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//   http ://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: matthewbstokes@gmail.com (Matthew Stokes)

// Common utils
#include "Utils.h"

using std::unordered_map;
using std::vector;
using std::string;
using std::ifstream;
using std::ofstream;
using std::set;
using std::fixed;
using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using Eigen::Vector3f;
using Eigen::Vector3d;
using Eigen::Matrix3f;

  void GetXYZEulerRotationsFromPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], Eigen::Vector3d& rot) {
    Matrix3f m;
    m << pose[0], pose[1], pose[2],
      pose[4], pose[5], pose[6],
      pose[8], pose[9], pose[10];

    rot = m.eulerAngles(1, 0, 2); // y,x,z  // fixed z,x,y
  }


namespace PrettyPrint {

  void PrintDelimiter(DELIMITER delim) {
    switch (delim) {
    case COMMA:
      printf(", ");
      break;
    case SPACE:
      printf(" ");
      break;
    }
  }

  void PrintEndline(ENDLINE end) {
    switch (end) {
    case NEWLINE:
      printf("\n");
      break;
    case TAB:
      printf("\t");
      break;
    case ENDCOMMA:
      printf(", ");
      break;
    }
  }

  void PrintJoints(const float(&joint_positions)[NUMBER_OF_JOINTS],
    ENDLINE end, DELIMITER delim) {
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
      printf("%.7f", DEG(joint_positions[i]));
      if (i != NUMBER_OF_JOINTS - 1) {
        PrintDelimiter(delim);
      }
    }
    PrintEndline(end);
  }

  void PrintJoints(vector<double>& joint_positions,
    ENDLINE end, DELIMITER delim) {
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
      printf("%.7f", DEG(joint_positions[i]));
      if (i != NUMBER_OF_JOINTS - 1) {
        PrintDelimiter(delim);
      }
    }
    PrintEndline(end);
  }

  void PrintLoad(const float(&lc_values)[LOAD_CELL_DOF],
    ENDLINE end, DELIMITER delim) {

    for (int i = 0; i < LOAD_CELL_DOF; ++i) {
      printf("%.2f", lc_values[i]);
      if (i != LOAD_CELL_DOF - 1) {
        PrintDelimiter(delim);
      }
    }
    PrintEndline(end);
  }

  void PrintLoad(load& l, ENDLINE end, DELIMITER delim) {
    printf("%d", l.t);
    PrintDelimiter(delim);
    printf("%.3f", l.x);
    PrintDelimiter(delim);
    printf("%.3f", l.y);
    PrintDelimiter(delim);
    printf("%.3f", l.z);
    PrintDelimiter(delim);
    printf("%.5f", l.mx);
    PrintDelimiter(delim);
    printf("%.5f", l.my);
    PrintDelimiter(delim);
    printf("%.5f", l.mz);
    PrintEndline(end);
  }

  void PrintLoads(vector<load>& loads, ENDLINE end, DELIMITER delim) {
    printf("t, x, y, z, mx, my, mz\n");
    for (std::vector<load>::iterator it = loads.begin(); it != loads.end(); ++it) {
      PrintLoad(*it, end, delim);
    }
  }

  void PrintPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end, DELIMITER delim) {
    for (int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
      if (i % 4 == 0 && i != 0) {
        printf("\n");
      }
      printf("%f", pose[i]);
      if (i != NUMBER_OF_FRAME_ELEMENTS - 1) {
        PrintDelimiter(delim);
      }
    }
    PrintEndline(end);
    PrintEndline(end);
  }

  void PrintPose(const double(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end, DELIMITER delim) {
    for (int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
      if (i % 4 == 0 && i != 0) {
        printf("\n");
      }
      printf("%.12f", pose[i]);
      if (i != NUMBER_OF_FRAME_ELEMENTS - 1) {
        PrintDelimiter(delim);
      }
    }
    PrintEndline(end);
  }

  void PrintHumanReadablePose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end, DELIMITER delim) {
    printf("Displacements (mm): x, y, z\n");
    printf("%.2f", pose[3] * 1000);   // x
    PrintDelimiter(delim);
    printf("%.2f", pose[7] * 1000);   // y
    PrintDelimiter(delim);
    printf("%.2f", pose[11] * 1000);  // z
    PrintEndline(end);
  }

  void PrintAbsoluteDifference(load& val_desired, load& val,
    ENDLINE end, DELIMITER delim) {

    load diff = { 0,
      val_desired.x - val.x,
      val_desired.y - val.y,
      val_desired.z - val.z,
      val_desired.mx - val.mx,
      val_desired.my - val.my,
      val_desired.mz - val.mz
    };
  }

  void PrintPrompt() {
    printf("\n> ");
  }

  char GetDelim(DELIMITER delim) {
    switch (delim) {
    case COMMA:
      return ',';
    case SPACE:
      return ' ';
    default:
      return '?';
    }
  }

  char GetEnd(ENDLINE end) {
    switch (end) {
    case NEWLINE:
      return '\n';
    case TAB:
      return '\t';
    case ENDCOMMA:
      return ',';
    default:
      return '?';
    }
  }
  void LogLoad(std::ofstream& os, load& l,
    ENDLINE end, DELIMITER delimiter) {
    char delim = GetDelim(delimiter);
    char endchar = GetEnd(end);
    os << l.x << delim << l.y << delim << l.z << delim <<
      l.mx << delim << l.my << delim << l.mz << endchar;
  }
  void LogLoad(std::ofstream& os, float(&load_val)[NUMBER_OF_CART_DOFS],
    ENDLINE end, DELIMITER delimiter) {
    char delim = GetDelim(delimiter);
    char endchar = GetEnd(end);
    for (unsigned int i = 0; i < NUMBER_OF_CART_DOFS - 1; ++i) {
      os << load_val[i] << delim;
    }
    os << load_val[NUMBER_OF_CART_DOFS - 1] << endchar;
  }

  void LogPosition(std::ofstream& os, float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end, DELIMITER delimiter) {
    char delim = GetDelim(delimiter);
    char endchar = GetEnd(end);
    os << pose[3] << delim << pose[7] << delim << pose[11] << endchar;
  }

  void LogPose(std::ofstream& os, float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end, DELIMITER delimiter) {
    char delim = GetDelim(delimiter);
    char endchar = GetEnd(end);
    for (unsigned int i = 0; i < NUMBER_OF_FRAME_ELEMENTS - 1; ++i) {
      os << pose[i] << delim;
    }
    os << pose[11] << endchar;
  }

}  // end namespace

namespace Utils {

  std::string exec(const char* cmd) {
    FILE* pipe = _popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe)) {
      if (fgets(buffer, 128, pipe) != NULL)
        result += buffer;
    }
    _pclose(pipe);
    return result;
  }

  void SLERP() {

    // START ///////////////////////
    Eigen::Matrix3d m;

    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

    printf("Original\n");
    std::cout << m << "\n";

    Eigen::Vector3d ea = m.eulerAngles(0, 1, 2);
    std::cout << "Euler angles" << "\n";
    std::cout << ea << std::endl << std::endl;

    Eigen::Quaterniond qa(m);
    //qa.normalize();
    //////////////////////////////////

    // End ///////////////////////////
    Eigen::Matrix3d n;

    // .5 = 90 deg
    n = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())  // x rotation
      * Eigen::AngleAxisd(0.33*M_PI, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ());

    printf("End\n");
    std::cout << n << "\n";

    Eigen::Vector3d ea2 = n.eulerAngles(0, 1, 2);
    std::cout << "Euler angles" << "\n";
    std::cout << ea2 << std::endl << std::endl;

    Eigen::Quaterniond qb(n);
    //qb.normalize();
    //////////////////////////////////


    // Rotation ///////////////////////
    printf("Rotation\n");
    Eigen::Matrix3d o;
    
    Eigen::Quaterniond qres;
    for (int t = 0; t < 50; ++t) {
      qres = qa.slerp(float(t) / 50.0, qb);
      ea = qres.matrix().eulerAngles(0, 1, 2);
      qres.normalize();
      //o = qres.toRotationMatrix();
      //ea = o.eulerAngles(0, 1, 2);
      printf("%.5f\t%.5f\t%.5f\n", DEG(ea[0]), DEG(ea[1]), DEG(ea[2]));
    }

    std::cout << "Euler angles END" << "\n";
    printf("%.5f\t%.5f\t%.5f\n", ea2[0], ea2[1], ea2[2]);
  }

  // http://www.cplusplus.com/forum/beginner/14349/
  void ToClipboard(const std::string &s){
    OpenClipboard(0);
    EmptyClipboard();
    HGLOBAL hg = GlobalAlloc(GMEM_MOVEABLE, s.size());
    if (!hg){
      CloseClipboard();
      return;
    }
    memcpy(GlobalLock(hg), s.c_str(), s.size());
    GlobalUnlock(hg);
    SetClipboardData(CF_TEXT, hg);
    CloseClipboard();
    GlobalFree(hg);
  }

  void ToClipboard(const float* arr, unsigned int length) {
    string s = "";
    for (unsigned int i = 0; i < length; ++i) {
      s+= std::to_string(arr[i]);
      s += ", ";
    }
    s.erase(s.end() - 2);  // remove last comma and space
    ToClipboard(s);
  }

  void ToClipboard(const double* arr, unsigned int length) {
    string s = "";
    for (unsigned int i = 0; i < length; ++i) {
      s += std::to_string(arr[i]);
      s += ", ";
    }
    s.erase(s.end() - 2);  // remove last comma and space
    ToClipboard(s);
  }

  // Generate a random assortment of letters and numbers 7 characters in length
  // Used as an identifier in the KUKA log file name so we can easily
  // find and rename the file later
  void GenerateFileIdentifier(char *identifier) {
    int length = 7;
    static const char alpha_num[] =
      "0123456789"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";
    for (int i = 0; i < length; ++i) {
      identifier[i] = alpha_num[rand() % (sizeof(alpha_num) - 1)];
    }
    identifier[length] = 0;
  }

  // Rename file with identifier from in the log_file_path directory
  // Searches in the log_file_path for a file whose name contains the identifier
  // string. If found changes the file name to log_file_name +
  // log_file_extension. Else returns ERROR_FILE_NOT_FOUND.
  int RenameLogFile(std::string log_file_name, const char *identifier,
    std::string log_file_path, std::string log_file_extension) {
    // get all files in current directory
    path p(log_file_path);
    directory_iterator end_iter;
    string filename;

    if (exists(p) && is_directory(p)) {
      bool file_found = false;
      for (directory_iterator iter(p); iter != end_iter; ++iter) {
        if (is_regular_file(iter->path())) {
          // if identifier found
          if (iter->path().filename().string().find(identifier) !=
            std::string::npos) {
            rename(iter->path(),
              path(log_file_path + log_file_name + log_file_extension));
            file_found = true;
          }
          std::cout << log_file_path + log_file_name + log_file_extension << "\n";
        }
      }
      if (!file_found) {
        fprintf(stderr, "ERROR, could not rename file; file not found");
        return ERROR_FILE_NOT_FOUND;
      }
    }
    return SUCCESS;
  }

  void SetLoad(const float(&load_cell_forces)[6], load& current_load) {
    current_load.x = load_cell_forces[0];
    current_load.y = load_cell_forces[1];
    current_load.z = load_cell_forces[2];
    current_load.mx = load_cell_forces[3];
    current_load.my = load_cell_forces[4];
    current_load.mz = load_cell_forces[5];
  }

  void SplitLine(string line, vector<string>& exploded_items, string delimiter) {
    boost::algorithm::split(exploded_items, line, boost::is_any_of(delimiter),
      boost::token_compress_on);
  }
}

namespace Parser {

  int ParseToolPath(std::string input_string, vector<Eigen::Vector3d>& path) {
    vector<string> lines;
    vector<string> elements;
    boost::algorithm::split(lines, input_string, boost::is_any_of("\n"),
      boost::token_compress_on);

    printf("Starting Reading\n");
    for (unsigned int i = 0; i < lines.size() - 1;  ++i) {
      boost::algorithm::split(elements, lines[i], boost::is_any_of(","),
        boost::token_compress_on);
      Eigen::Vector3d point;
      point << stof(elements[0])/1000, stof(elements[1])/1000, stof(elements[2])/1000;
      path.push_back(point);
    }
    printf("Finished Reading\n");
    return 0;
  }

  int ParsePontsPathFromSplineFile(string input_file, float(&origin)[3], vector<vector<float>>& path) {
    ifstream input(input_file);
    float x;
    float y;
    float z;
    while (input >> x >> y >> z) {
      path.push_back({ x + origin[0], y + origin[1], z + origin[2] });
    }
    return 0;
  }

  void ListPointsInLoadThreshold(string loads_input_file, float (&load_threshold)[LOAD_CELL_DOF], float(&pose_threshold)[NUMBER_OF_CART_DOFS], string output_file) {
    vector<vector<float>> points;
    // open file(s)
    ifstream loads_file(loads_input_file);
    ofstream os(output_file);

    // read in points (assume no header and comma seperated)
    string line;
    vector<string> split_line;
    while (loads_file >> line) {
      Utils::SplitLine(line, split_line, ",");
      vector<float> row;
      for (unsigned int i = 1; i < split_line.size(); ++i) {  // ignore t      
        row.push_back(std::stof(split_line[i]));
      }
      points.push_back(row);
    }

    // O(n^2) create list of points within threshold
    bool in_force_threshold;
    bool in_pose_threshold;
    bool written_point;
    unordered_map<unsigned int, set<unsigned int>> matches;

    // i is current point to check all other points against
    for (unsigned int i = 0; i < points.size(); ++i) {
      written_point = false;
      for (unsigned int j = 0; j < points.size(); ++j) {
        in_force_threshold = true;
        in_pose_threshold = true;

        // skip check against yourself
        if (i == j) {
          continue;
        }

        // check against load threshold for each parameter
        for (unsigned int k = 0; k < LOAD_CELL_DOF; ++k) {
          if (abs(points[i][k] - points[j][k]) > load_threshold[k] ) {
            in_force_threshold = false;
            break;
          }
        }

        // check distance and rotation threshold for matching point
        if (in_force_threshold) {
          for (unsigned int k = LOAD_CELL_DOF; k < LOAD_CELL_DOF * 2; ++k) {
            if (abs(points[i][k] - points[j][k]) > pose_threshold[k% LOAD_CELL_DOF]) {
              // check euler angle 0->180 swap edge case
              if (abs(points[i][k] - points[j][k]) > 175) {
                continue;
              }
              in_pose_threshold = false;
              break;
            }
          }
          if (!in_pose_threshold) {
            // check map for inverse element in map
            unordered_map<unsigned int, set<unsigned int >>::const_iterator found = matches.find(j);
            if (found != matches.end()) {
              if (found->second.find(i) == found->second.end()) {
                // element is not found in the pairing so add to self
                unordered_map<unsigned int, set<unsigned int >>::const_iterator first = matches.find(i);
                // create set if necessary and add element
                if (first == matches.end()) {
                  matches[i] = set<unsigned int>();
                }
                matches[i].insert(j);
              }
            }
            else {  // key j not found in matches therefore empty set
              unordered_map<unsigned int, set<unsigned int >>::const_iterator first = matches.find(i);
              // create set if necessary and add element
              if (first == matches.end()) {
                matches[i] = set<unsigned int>();
              }
              matches[i].insert(j);
            }
          }  // endif not in pose threshold
        }  // end if not in force threshold
      }
    }
    // print matches to output file
    for (auto kv : matches) {
      os << kv.first << ": ";
      for (unsigned int i : kv.second) {
        os << i << " ";
      }
      os << "\n";
    }
  }
  
  void GetScalingFactorsFromCSV(string follow_loads_path,
    vector<load>& scaling_factors) {

    vector<vector<float>> points;
    // open file(s)
    ifstream loads_file(follow_loads_path);

    // read in points (assume no header and comma seperated)
    string line;
    vector<string> split_line;
    while (loads_file >> line) {
      Utils::SplitLine(line, split_line, ",");
      vector<float> row;
      for (unsigned int i = 1; i < split_line.size() - 1; ++i) {  // ignore t      
        row.push_back(std::stof(split_line[i]));
      }
      points.push_back(row);
    }

    // generate 2D scaling factors from current - previous forces wrt displacements
    // input file:
    //    I, Fx, Fy, Fz, Mx, My, Mz, Dx, Dy, Dz, Rx, Ry, Rz
    //    where: Rx, Ry, Rz are from Euler X,Y,Z rotation m.eulerAngle(0,1,2)
    // assuming:
    //    TCP Fx -> Dz, TCP Fy -> Dy, TCP Dz -> Dx
    //    TCP Mx -> Rz, TCP My -> Ry, TCP Rz -> Rx
    // scaling factors (delta):
    //    Sx = Dz/Fx, Sy = Dy/Fy, Sz = Dx/Fz
    //    Smx = Rz/Mx, Smy = Ry/My, Smz = Rx/Mz
    vector<float> prev;

    // loop over all points, calculate scaling value based on prev point
    // and store into scaling factors vector
    for (unsigned int i = 1; i < points.size(); ++i) {
      load scaling_factor;
      scaling_factor.t = i - 1;
      scaling_factor.x = ((points[i][9] - points[i - 1][9]) /
        (points[i][1] - points[i-i][1]));
      scaling_factor.y = ((points[i][8] - points[i - 1][8]) /
        (points[i][2] - points[i - i][2]));
      scaling_factor.z = ((points[i][7] - points[i - 1][7]) /
        (points[i][3] - points[i - i][3]));
      scaling_factor.mx = ((points[i][12] - points[i - 1][12]) /
        (points[i][4] - points[i - i][4]));
      scaling_factor.my = ((points[i][11] - points[i - 1][11]) /
        (points[i][5] - points[i - i][5]));
      scaling_factor.mz = ((points[i][10] - points[i - 1][10]) /
        (points[i][6] - points[i - i][6]));

      // scale scaling factors by factor of 4
      scaling_factors.push_back(scaling_factor);
    }
  }
}  // end namespace

namespace Logging {

  void WriteLoadAndPosition(std::ofstream& os, load l, float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
    Vector3d rot;

    GetXYZEulerRotationsFromPose(pose, rot);
    os << l.t << "," <<
      l.x << "," << l.y << "," << l.z << "," <<
      l.mx << "," << l.my << "," << l.mz << "," <<
      pose[3] << "," << pose[7] << "," << pose[11] << "," <<
      rot[0] << "," << rot[1] << "," << rot[2] << "\n";
  }

  void WritePose(std::ofstream& of, float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
    for (unsigned int i = 0; i < 11; ++i) {
      of << pose[i] << ",";
    }
    of << pose[11] << ",";
  }


  int WriteLoadsAndPositionsStream(ofstream& of, std::vector<load>& loads, std::vector<load>& pos_rot) {
    for (unsigned int i = 0; i < loads.size(); ++i) {
      of << loads[i].t << "," <<
        loads[i].x << "," << loads[i].y << "," << loads[i].z << "," <<
        loads[i].mx << "," << loads[i].my << "," << loads[i].mz << "," <<
        pos_rot[i].x << "," << pos_rot[i].y << "," << pos_rot[i].z << "," <<
        pos_rot[i].mx << "," << pos_rot[i].my << "," << pos_rot[i].mz << "\n";
    }
    return SUCCESS;
  }

  void WriteLoadsAndPositions(ofstream& of, std::vector<std::vector<float>> loads,
    std::vector<std::vector<double>> pos_rot) {
    unsigned int count = 0;
    for (unsigned int i = 0; i < loads.size(); ++i) {
      of << count++ << ",";
      for (unsigned int j = 0; j < loads[i].size(); ++j) {
        of << loads[i][j] << ",";
      }
      for (unsigned int j = 0; j < pos_rot[i].size() - 1; ++j) {
        of << pos_rot[i][j] << ",";
      }
      of << pos_rot[i][pos_rot[i].size() - 1] << "\n";
    }
  }
}

namespace Reader {
  static const unsigned int ERROR_INVALID_INPUT_FILE = 1;
  // Get Fx, Fy, Fz, Mx, My, Mz from logged force file
  int GetLoadsFromCSV(string loads_file_path, vector<vector<double>>& loads)  {
    std::ifstream loads_input_file(loads_file_path);
    if (!loads_input_file.is_open()) {
      printf("Unable to open forces input file");
      return ERROR_INVALID_INPUT_FILE;
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

  int GetPositionsFromCSV(string positions_file_path, vector<vector<double>>& positions) {
    std::ifstream loads_input_file(positions_file_path);
    if (!loads_input_file.is_open()) {
      printf("Unable to open forces input file");
      return ERROR_INVALID_INPUT_FILE;
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

  int GetToolDirectionPathFromCSV(const string positions_file_path, vector<Direction>& tool_path,
    Eigen::Matrix4d& start_frame) {
    std::ifstream loads_input_file(positions_file_path);
    if (!loads_input_file.is_open()) {
      printf("Unable to open forces input file");
      return ERROR_INVALID_INPUT_FILE;
    }

    Direction direction;
    vector<Vector3d> positions;
    Eigen::Matrix3d tool_orientation;
    tool_orientation <<
      start_frame(0, 0), start_frame(0, 1), start_frame(0, 2),
      start_frame(1, 0), start_frame(1, 1), start_frame(1, 2),
      start_frame(2, 0), start_frame(2, 1), start_frame(2, 2);

    string line;
    vector<string> line_split_loads;
    while (std::getline(loads_input_file, line)) {
      boost::algorithm::split(line_split_loads, line, boost::is_any_of(","),
        boost::token_compress_on);
      if (line_split_loads.size() == 1) {
        break;
      }
      Vector3d position(
        stod(line_split_loads[7]),
        stod(line_split_loads[8]),
        stod(line_split_loads[9]));
      positions.push_back(tool_orientation*position);
    }
    for (unsigned int i = 0; i < positions.size() - 1; ++i) {
      // direction
      Vector3d tmp = positions[i + 1] - positions[i];
      if (abs(tmp.x()) > abs(tmp.y()) && abs(tmp.x()) > abs(tmp.z())) {
        direction.axis = X;
        if (tmp.x() > 0)
          direction.sign = POS;
        else
          direction.sign = NEG;
      }
      else if (abs(tmp.y()) > abs(tmp.x()) && abs(tmp.y()) > abs(tmp.z())) {
        direction.axis = Y;
        if (tmp.y() > 0)
          direction.sign = POS;
        else
          direction.sign = NEG;
      }
      else {
        direction.axis = Z;
        if (tmp.z() > 0)
          direction.sign = POS;
        else
          direction.sign = NEG;
      }
      DBGPRINT("dir %d, sign: %d", direction.axis, direction.sign);
      tool_path.push_back(direction);
    }
    
    return SUCCESS;
  }
}