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

// Config.h contains global static variables and configuration settings 
#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>
#include <vector>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA-LWR\external\eigen\Eigen\Dense>
#import "C:\Users\HMMS\Documents\Corey\Hexapod\Hexapod Code\bin\x64\HexapodCOM.dll"

#ifndef PI
#define PI  3.14159265358979323846264338327950288419
#endif

#ifndef DEG
#define DEG(A) ((A)*180.0/PI)
#endif

#ifndef RAD
#define RAD(A) ((A)*PI/180.0)
#endif

#define DEBUG true

#define FILENAME (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#define DBGVAR( os, var ) \
  if (DEBUG) (os) << "DBG " << FILENAME << "(" << __LINE__ << ") "\
    << __FUNCTION__ << #var << ": " << (var) << "\n"

#define DBGPRINT(M, ...) \
  if (DEBUG) fprintf(stderr, "DBG %s(%d) " M "\n", FILENAME, __LINE__, ##__VA_ARGS__)

const unsigned int NUMBER_OF_FRAME_ELEMENTS = 12;
const unsigned int NUMBER_OF_ROTATION_ELEMENTS = 9;
const unsigned int NUMBER_OF_JOINTS = 7;
const unsigned int NUMBER_OF_CART_DOFS = 6;
const unsigned int LOAD_CELL_DOF = 6;

const unsigned int SUCCESS = 0;
const unsigned int EOK = 0;

enum ControlMode { JOINT, CARTESIAN };
enum RecordStopCondition { A0_ANGLE, TIME, KEY_PRESS };

// relative tool path structure
enum Axis { X, Y, Z };
enum Sign { POS, NEG };
struct Direction {
public:
  Axis axis;
  Sign sign;
};

// #TODO (matthewbstokes) remove this concept. Replace with vector.
struct load {
  int t;
  float x;
  float y;
  float z;
  float mx;
  float my;
  float mz;
};

/*
Configuration specific properties including:
  - desktop specific filepaths
  - common file extensions
  - load cell and LWR configuration settings
  - LWR starting frame
*/
namespace Config {
  // load cell sample rate in Hz
  const float SAMPLE_RATE = 5000;
  // load cell USB channel
  const std::string LOADCELL_CHANNEL = "Dev4/ai0:5"; //"Dev1/ai0:5" USB-6211
  // Load cell -> TCP transformation (mm)
  //   transformation: displacements and rotations in the order
  //                     Dx, Dy, Dz, Rx, Ry, Rz
  const float LOAD_CELL_TRANSFORMATION[LOAD_CELL_DOF] =
	  //{ 37.211f, 0.0f, 185.752f, 0.0f, 0.0f, 0.0f };  
	  //{ 91.73f, 0.0f, 153.0f, 0.0f, 0.0f, 0.0f } square brass; 
	  //{ 37.211f, 0.0f, 154.252f, 0.0f, 0.0f, 0.0f } V white acrylic w/ white robot; 
	  //{ 37.211f, 47.0f, 185.752f, -22.5f, 0.0f, 0.0f }
	  { 91.73f, 0.0f, -90.0f, 0.0f, 0.0f, 0.0f }; //ear square brass to tool tip old extruder printer holder;
	  //{ 93.73f, 0.0f, -88.13f, 0.0f, 0.0f, 0.0f }; //ear square brass to tool tip;
	  //{ 0.209f, 0.0f, 7.82f, 0.0f, 0.0f, 0.0f }; //ear square brass;

  // starting LWR tool pose (should be consistent with the KRC
  // http://stackoverflow.com/questions/31549398/c-eigen-initialize-static-matrix/31549780#31549780
  // registration frame
  static Eigen::Matrix4d start_frame = [] {
    Eigen::Matrix4d tmp;
    tmp << 
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
    return tmp;
  }();

  // desktop skecific file paths
  // #TODO (matthewbstokes) change to relative paths
  namespace Filepath {
    // file folders and base path
    const std::string BASE_PATH = \
      "C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA-LWR\\";
    const std::string EXP_PATH = BASE_PATH + "experiments\\";
    const std::string LOGS_PATH = BASE_PATH + "experiments\\free_record\\";
    const std::string PATHS_POSITION_PATH = BASE_PATH + "paths\\position\\";
    const std::string PATHS_LOAD_PATH = BASE_PATH + "paths\\load\\";
    const std::string ASCII_PATH = BASE_PATH + "misc\\";

    // load cell calibration
    const std::string LOAD_CELL_CALIBRATION_PATH = \
      "../../calibration/FT17252.cal";
    // FRI calibration
    const std::string FRI_INIT = \
      "../../calibration/980039-FRI-Driver.init";
    // PyCAM Tool path parser and python interpreter
    const std::string python_path = \
      "C:\\Users\\HMMS\\Downloads\\WinPython-64bit-3.4.3.3\\python-3.4.3.amd64\\python.exe";
    const std::string parser_path = \
      "\"C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA-LWR\\gui\\ngc_parser.py\"";
    const std::string plot_gen = \
      "\"C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA-LWR\\gui\\plot_graphs.py\"";
  };

  // common file extensions
  namespace Extension{
    const std::string CSV = ".csv";
    const std::string DAT = ".dat";
    const std::string TXT = ".txt";
  };
};

namespace Errors {
  const unsigned int ERROR_INCORRECT_NUM_ELEMENTS = 1;
  const unsigned int ERROR_INVALID_SELECTION = 2;
  const unsigned int ERROR_INVALID_INPUT_FILE = 1;
  const unsigned int ERROR_MACHINE_NOT_OKAY = 2;
  const unsigned int SUCCESS = 0;
  const unsigned int ERROR_CSV_LOADS = 2;
  const unsigned int ERROR_LOAD_CELL_CALIBRATION = 1;
  const unsigned int ERROR_DAQ_START = 2;
  const unsigned int ERROR_NO_FILE_SET = 2;
  const unsigned int ERROR_TOOL_TRANSFORMATION = 3;
  const unsigned int ERROR_TIMEOUT_COUNT = 3;
  const unsigned int ERROR_OUT_OF_BOUNDS = 4;
};

#endif  // CONFIG_H_
