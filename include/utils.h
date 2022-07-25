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

// Utils.h is a collection of common project utils
#ifndef UTILS_H_
#define UTILS_H_

#define NOMINMAX
#include <windows.h>
#include "Config.h"
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <fstream>
#include <math.h> 

#include <unordered_map>

#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\numeric\ublas\io.hpp> 
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\numeric\ublas\matrix.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\numeric\ublas\lu.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\filesystem.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\algorithm\string.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\algorithm\string\join.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\algorithm\string\split.hpp>

#include <Eigen/Dense>

// generator/continuation for C++
// author: Andrew Fedoniouk @ terrainformatica.com
// idea borrowed from: "coroutines in C" Simon Tatham,
//   http://www.chiark.greenend.org.uk/~sgtatham/coroutines.html
struct _generator
{
  int _line;
  // bottom, top, step
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  int size;
  _generator() :_line(0) {}
};

#define $generator(NAME) struct NAME : public _generator

#define $emit(T) bool operator()(T& _rv) { \
                    switch(_line) { case 0:;

#define $stop  } _line = 0; return false; }

#define $yield(V)     \
        do {\
            _line=__LINE__;\
            _rv = (V); return true; case __LINE__:;\
                        } while (0)

$generator(gains_combinatoric)
{
  std::vector<float> v;
  float i;
  float j;
  float k;
  $emit(std::vector<float>)
  v.push_back(0); v.push_back(0); v.push_back(0);
  size = abs((static_cast<int>((x[1] - x[0]) / x[2]) + 1) * 
    (static_cast<int>((y[1] - y[0]) / y[2]) + 1) * 
    (static_cast<int>((z[1] - z[0]) / z[2]) + 1));
  for (i = x[0]; i <= x[1]; i += x[2]) {
    v[0] = i;
    for (j = y[0]; j <= y[1]; j += y[2]) {
      v[1] = j;
      for (k = z[0]; k <= z[1]; k += z[2]) {
        v[2] = k;
        $yield(v); // a.k.a. yield in Python
      }
    }
  }
  $stop; // stop, end of sequence. End of body of the generator.
};

void GetXYZEulerRotationsFromPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], Eigen::Vector3d& rot);

namespace PrettyPrint {
  enum ENDLINE { NEWLINE, TAB, NONE, ENDCOMMA };
  enum DELIMITER { COMMA, SPACE };
  
  void PrintDelimiter(DELIMITER delim);
  void PrintEndline(ENDLINE end);

  void PrintJoints(std::vector<double>& joint_positions,
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);
  void PrintJoints(const float(&joint_positions)[NUMBER_OF_JOINTS],
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);

  void PrintLoad(const float(&lc_values)[LOAD_CELL_DOF],
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);

  void PrintLoad(load& l, ENDLINE end=NEWLINE, DELIMITER delim=COMMA);

  void PrintLoads(std::vector<load>& loads,
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);

  void PrintPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);
  void PrintPose(const double(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end = NEWLINE, DELIMITER delim = COMMA);

  void PrintHumanReadablePose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);

  void PrintAbsoluteDifference(load& val_desired, load& val,
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);

  void PrintTestCase(std::string test_case_name, std::string test_item,
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);

  void PrintPrompt();

  void LogLoad(std::ofstream& os, load& l,
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);
  void LogLoad(std::ofstream& os, float(&load)[NUMBER_OF_CART_DOFS],
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);
 // void LogLoad(std::ofstream& os, unsigned int point, std::vector<std::vector<double>> &load,
 //   ENDLINE end, DELIMITER delimiter);

  void LogPosition(std::ofstream& os, float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end = NEWLINE, DELIMITER delim = COMMA);

  void LogPose(std::ofstream& os, float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    ENDLINE end=NEWLINE, DELIMITER delim=COMMA);
};

namespace Utils {
  void ToClipboard(const std::string &s);
  void ToClipboard(const float* arr, unsigned int length);
  void ToClipboard(const double* arr, unsigned int length);

  // Generate a random assortment of letters and numbers 7 characters in length
  // Used as an identifier in the KUKA log file name.
  // Parameters:
  //   identifier: the uniquie identifier
  void GenerateFileIdentifier(char *identifier);

  // Rename file with identifier from in the log_file_path directory to
  // log_file_name + log_file_extension.
  // Parameteres:
  //   log_file_name: desired name for the log file.
  //   identifier: unique log file identifier. Some character string that is
  //               unique to a single log file.
  //   log_file_path: log directory path. E.g. "C:\User\Desktop\LogFolder\".
  //   log_file_extension: extension for the log file. E.g. ".dat".
  int RenameLogFile(std::string log_file_name, const char *identifier,
    std::string log_file_path, std::string log_file_extension);

  void SLERP();

  std::string exec(const char* cmd);

  void SetLoad(const float(&load_cell_forces)[6], load& current_load);

  void SplitLine(std::string line, std::vector<std::string>& exploded_items, std::string delimiter);

  template <typename T>
  T clip(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
  }
};

namespace Parser {
  int ParsePontsPathFromSplineFile(std::string input_file, float(&origin)[3], std::vector<std::vector<float>>& path);
  void ListPointsInLoadThreshold(std::string loads_input_file,
    float(&threshold)[LOAD_CELL_DOF],
    float(&pose_threshold)[NUMBER_OF_CART_DOFS], std::string output_file);

  void GetScalingFactorsFromCSV(std::string follow_loads_path,
    std::vector<load>& scaling_factors);

  int ParseToolPath(std::string input_file, std::vector<Eigen::Vector3d>& path);

};

namespace Logging {
  void WritePose(std::ofstream& of, float(&pose)[NUMBER_OF_FRAME_ELEMENTS]);
  void WriteLoadAndPosition(std::ofstream& os, load l,
    float(&pose)[NUMBER_OF_FRAME_ELEMENTS]);
  int WriteLoadsAndPositionsStream(std::ofstream& of, std::vector<load>& loads, std::vector<load>& pos_rot);
  void WriteLoadsAndPositions(std::ofstream& of, std::vector<std::vector<float>> loads,
    std::vector<std::vector<double>> pos_rot);
};

namespace Reader {
  int GetLoadsFromCSV(std::string loads_file_path, std::vector<std::vector<double>>& loads);
  int GetPositionsFromCSV(std::string positions_file_path,
    std::vector<std::vector<double>>& positions);

  // Travel direction is calculated from the pre-planned path as an independent
  // X, Y, or Z translation wrt. the fixation/object coordinate system.
  // The pre-planned travel path should be discretized into singular independent
  // directions with spacing 1mm between path points.
  // E.g.
  //   point 1: (0, 0, 0)
  //   point 2: (0, 1, 0)
  //   point 3: (0, 1, 1)
  //   point 4: (0, 2, 1)
  //
  // Important to note is that the travel direction is relative/fixed to the
  // objects/patients coordinate system and thus any movement of the
  // object/patient does not corrupt results.
  // 
  // Parameters:
  //   positions: Positional path wrt. the object/patients coordinate system
  //              located at the center of the object/patient fixation.
  //   path_point: Current location/point along the planned path.
  //   direction: Direction of positional travel between the current path
  //              point and next path point. Direction is a struct which has
  //              both a value (X=0, Y=1, Z=2) and sign (+, -).
  int GetToolDirectionPathFromCSV(std::string positions_file_path, std::vector<Direction>& tool_path,
    Eigen::Matrix4d& start_frame);

};

  

#endif  // UTILS_H_
