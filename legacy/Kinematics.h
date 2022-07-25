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

const unsigned int NUMBER_OF_ROTATION_ELEMENTS = 9;
const unsigned int NUMBER_OF_FRAME_ELEMENTS = 12;
const unsigned int NUMBER_OF_SCALING_FACTORS = 6;
const unsigned int NUMBER_OF_CART_DOFS = 6;
const unsigned int NUMBER_OF_JOINTS = 7;
const unsigned int LOAD_CELL_DOF = 6;
const unsigned int SUCCESS = 0;

const std::string CSV = ".csv";
const std::string DAT = ".dat";
const std::string TXT = ".txt";

enum AngleUnit { DEG, RAD };
struct load {
  int t;
  float x;
  float y;
  float z;
  float mx;
  float my;
  float mz;
};

namespace Config {
  const float SAMPLE_RATE = 2000;  // hz
  const float MAX_DISPLACEMENT_PER_STEP = 0.0006f;  // mm
  const float MAX_ROTATION_PER_STEP = 0.01f;  // rad
  const std::string LOADCELL_CHANNEL = "Dev1/ai0:5";
  const std::string LOAD_CELL_CALIBRATION_PATH = \
    "../../misc/FT13036Nano25ReCalibrated.cal";
  static const std::string LOG_FILEPATH = "../../logs/";
  const std::string FRI_INIT = \
    "../../external/FRILibrary/etc/980039-FRI-Driver.init";

  // Load cell -> TCP transformation mm, deg validated with SW and three.js
  // 
  const float LOAD_CELL_TRANSFORMATION[LOAD_CELL_DOF] = \
    { -73.73f, 125.0f, 91.940f, 0.0f, 0.0f, 0.0f }; 
  // unloaded, unattached, no wire
  const float LOAD_CELL_BASELINE_BIAS[LOAD_CELL_DOF] = \
    { 
      //5.08671f, 1.00645f, -25.9693f, 3.22508f, 1.07249f, 0.928035
      4.16222143173218f, 3.50110197067261f, -19.2817325592041f, 2.74187064170837f, 0.786752343177795f, 0.95879191160202f
  };
  // from unload robot information
  const float KUKA_LOAD_CELL_BASLINE[NUMBER_OF_CART_DOFS] = \
    { 0.676f, -0.368f, 0.180f, 0.123f, -0.309f, 0.051f };
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
