// Copyright 2015 Matthew Stokes. All Rights Reserved.

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

// The interface LoadCell.h encapsulates load cell functionality.
//#ifndef KUKA_LWR_INCLUDE_LOADCELL_H_
//#define KUKA_LWR_INCLUDE_LOADCELL_H_

#ifndef DEBUG
#define DEBUG false
#endif
#ifndef FILENAME
#define FILENAME (strrchr(__FILE__, '\\') \
  ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#endif
#ifndef DBGVAR
#define DBGVAR(os, var) \
  if (DEBUG) (os) << "DBG " << FILENAME << "(" << __LINE__ << ") "\
    << __FUNCTION__ << #var << ": " << (var) << "\n"
#endif
#ifndef DBGPRINT
#define DBGPRINT(M, ...) \
  if (DEBUG) fprintf(stderr, "DBG %s(%d) " M "\n", \
    FILENAME, __LINE__, ##__VA_ARGS__)
#endif

#include <string>

class LoadCell {
 public:
  const std::string CALIBRATE_STR = "Calibrating load cell";
  const std::string TRANSFORM_STR = "Transforming load cell";
  const std::string START_STR = "Starting load cell";
  const std::string BIAS_STR = "Biasing load cell";
  const std::string STOP_STR = "Stopping load cell";

  const unsigned int SUCCESS = 0;
  const unsigned int ERROR_CALIBRATION = 1;
  const unsigned int ERROR_DAQ_START = 2;
  const unsigned int ERROR_TRANSFORMATION = 3;
  const unsigned int ERROR_DAQ_NOT_STARTED = 4;
  const unsigned int ERROR_BIAS_NOT_SET = 5;

  LoadCell() {}
  virtual ~LoadCell() {}

  virtual int Initialize(const std::string calibration_path,
    const float* transform, const float sample_rate, const std::string channel,
    const bool set_filter) = 0;

  virtual int Start(const float sample_rate, const std::string channel) = 0;
  virtual void Stop() = 0;

  virtual int SetCalibration(const std::string calibration_file_path) = 0;
  virtual int SetTransformation(const float* transform) = 0;
  virtual int SetBias() = 0;

  virtual int GetBias(float* bias) const = 0;
  virtual int GetVoltages(float *voltages) const = 0;
  virtual int GetLoads(float *loads) = 0;
};
//#endif  // KUKA_LWR_INCLUDE_LOADCELL_H_
