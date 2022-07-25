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

// Nano25E.cpp is intended for use with the ATINano25E load cell and
// Nation Instruments USB-6211 DAQ.
// Use the provided manufactures calbration script (.cal) when calibrating.
// This library bridges the ATICombinedDAQ library for interactions with the
// ATI load cell, and the DAQmx library for interactions with the DAQ device.

#include "Nano25E.h"
#include <string>
#include "DspFilters/Dsp.h"
#include <fstream>

namespace NanoLoadCell {
  #define DAQmxErrChk(functionCall) \
    if (DAQmxFailed(error = (functionCall))) goto Error;

  using DAQFTCLIBRARY::destroyCalibration;
  using DAQFTCLIBRARY::createCalibration;
  using DAQFTCLIBRARY::SetToolTransform;
  using DAQFTCLIBRARY::Bias;
  using DAQFTCLIBRARY::SetForceUnits;
  using DAQFTCLIBRARY::SetTorqueUnits;
  using DAQFTCLIBRARY::ConvertToFT;
  using std::string;

  int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle,
    int32 everyNsamplesEventType, uInt32 nSamples, void *callback_package) {
    int32 error = 0;
    char errBuff[2048] = { '\0' };
    int32 read = 0;
    
    // unpackage callback package
    CallbackPackage* package = static_cast<CallbackPackage*>(callback_package);

    DAQmxErrChk(DAQmxReadAnalogF64(
      taskHandle,                           // task
      SAMPLES_PER_CHANNEL,                  // samples per channel
      DAQ_TIMEOUT,                          // timeout
      DAQmx_Val_GroupByChannel,             // fill mode
      package->buffer,                      // array to read samples into
      SAMPLES_PER_CHANNEL * DOF,            // size of array
      &read,                                // samples per channel read
      NULL));                               // reserved

    // Voltage array's samples are grouped by channel.
    // e.g.
    //   10 SAMPLES_PER_CHANNEL grouped by channel
    //   transducer_0    transducer_1   ...  transducer_6
    //   [0, 1 ..., 9,   10, 11 ..., 19 ...  50, 51 ..., 59]    
    if (read > 0) {
      // get most recent values and put into raw voltages
      for (unsigned int i = 0; i < DOF; ++i) {
        package->raw_voltages[i] = package->buffer[(SAMPLES_PER_CHANNEL*(i+1))-1];
      }
      
      // filtering
      if (package->filtering_enabled) {
        // first pass
        package->filter->process(
          SAMPLES_PER_CHANNEL,
          make_2d<SAMPLES_PER_CHANNEL,
          SAMPLES_PER_CHANNEL*DOF>(package->buffer).data());

        // reverse array
        for (unsigned int i = 0; i < DOF; ++i) {
          std::reverse(package->buffer + (SAMPLES_PER_CHANNEL * i),
            package->buffer + (SAMPLES_PER_CHANNEL*(i+1) - 1));
        }

        // second pass
        package->filter->process(
          SAMPLES_PER_CHANNEL,
          make_2d<SAMPLES_PER_CHANNEL,
          SAMPLES_PER_CHANNEL*DOF>(package->buffer).data());
      }
      
      // average
      if (package->channel_averaging_enabled == true) {
        double sum = 0;
        for (unsigned int i = 0; i < DOF; ++i) {
          sum = 0;
          for (unsigned int j = 0; j < SAMPLES_PER_CHANNEL; ++j) {
            sum += package->buffer[(i*SAMPLES_PER_CHANNEL)+j];
          }
          package->voltages[i] = static_cast<float>(sum / SAMPLES_PER_CHANNEL);
        }
      }
      else {
        for (unsigned int i = 0; i < DOF; ++i) {
          package->voltages[i] = static_cast<float>(package->buffer[i * 10]);
        }
      }
    } // done read

    Error:
      if (DAQmxFailed(error)) {
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);
        printf("DAQmx Error: %s\n", errBuff);
      }
      return 0;
  }

  Nano25E::Nano25E() : task_handle_(0), voltages_(), raw_voltages_(), bias_(), cal_(NULL),
    lower_threshold_set_flag(true), bias_set_flag(true) {}

  Nano25E::~Nano25E() {
    // ATI calibration
    if (cal_ != NULL) {
      destroyCalibration(cal_);
    }
    // DAQmx tasks
    if (task_handle_ != 0) {
      DAQmxStopTask(task_handle_);
      DAQmxClearTask(task_handle_);
    }
  }


  // Calibrates, transforms, starts, and bias
  int Nano25E::Initialize(std::string calibration_path, const float* transform,
    const float sample_rate, std::string channel, bool set_filter) {

    unsigned int err_val = SUCCESS;
    if (calibration_path != "") {
      err_val = SetCalibration(calibration_path.c_str());
      if (err_val != SUCCESS) {
        return ERROR_CALIBRATION;
      }
    }
    if (transform != NULL) {
      err_val = SetTransformation(transform);
      if (err_val != SUCCESS) {
        return ERROR_TRANSFORMATION;
      }
    }
    if (sample_rate != 0 && channel != "") {
      err_val = Start(sample_rate, channel.c_str());
      if (err_val != SUCCESS) {
        return ERROR_DAQ_START;
      }
    }
    if (set_filter) {
      SetFilter();
    }
    return SUCCESS;
  }


  // Starts a thread to begin reading input from the DAQmx device at the
  // input sample rate. The options have been fine turned for the ATI Nano25E
  // load cell connected to a NI 8211 DAQ. Channel == "Dev1/ai0:5".
  int Nano25E::Start(const float sample_rate, const std::string channel) {
    int32 error = 0;
    char errBuff[2048] = { '\0' };
    DBGPRINT("%s", START_STR);

    // package callback data
    callback_.buffer = buffer_;
    callback_.filtering_enabled = filter_enabled_;
    callback_.filter = &filter_;
    callback_.voltages = voltages_;
    callback_.raw_voltages = raw_voltages_;
    callback_.channel_averaging_enabled = true;

    // configure DAQmx
    DAQmxErrChk(DAQmxCreateTask("", &task_handle_));
    DAQmxErrChk(DAQmxCreateAIVoltageChan(
      task_handle_,                     // task
      channel.c_str(),                  // physical channel
      "",                               // assigned channel name
      DAQmx_Val_Diff,                   // input terminal configuration
      MIN_VOLTAGE_VALUE,                // min val
      MAX_VOLTAGE_VALUE,                // max val
      DAQmx_Val_Volts,                  // units
      NULL));                           // custom scale name
    DAQmxErrChk(DAQmxCfgSampClkTiming(
      task_handle_,                     // task
      "",                               // use internal clock
      sample_rate,                      // sampling rate (Hz)
      DAQmx_Val_Rising,                 // active edge
      DAQmx_Val_ContSamps,              // sample continuously
      SAMPLES_PER_CHANNEL));            // samples per channel
    DAQmxErrChk(DAQmxRegisterEveryNSamplesEvent(
      task_handle_,                     // task
      DAQmx_Val_Acquired_Into_Buffer,   // type of event to receive
      SAMPLES_TILL_EVENT,               // number of samples until event
      0,                                // options
      EveryNCallback,                   // callback function
      &callback_));                     // callback data (struct)

    DAQmxErrChk(DAQmxStartTask(task_handle_));
    sample_rate_ = sample_rate;

    return SUCCESS;

  Error:
    if (DAQmxFailed(error))
      DAQmxGetExtendedErrorInfo(errBuff, 2048);
    if (task_handle_ != 0) {
      DAQmxStopTask(task_handle_);
      DAQmxClearTask(task_handle_);
    }
    if (DAQmxFailed(error)) {
      printf("DAQmx Error: %s\n", errBuff);
    }
    return ERROR_DAQ_START;
  }

  // Stops the DAQmx device/thread.
  void Nano25E::Stop() {
    DBGPRINT("%s", STOP_STR);
    if (task_handle_ != 0) {
      DAQmxStopTask(task_handle_);
      DAQmxClearTask(task_handle_);
    }
    task_handle_ = 0;
    sample_rate_ = 0;
  }

  void Nano25E::SetFilter(unsigned int order_number, unsigned int cutoff_freq,
    const bool enable_filter) {
    filter_.setup(FILTER_ORDER_NUMBER, sample_rate_, cutoff_freq);
    filter_enabled_ = enable_filter;
  }

  void Nano25E::EnableFilter(bool status) {
    filter_enabled_ = status;
  }

  // Loads calibration info for a transducer. Units are N, and N-m.
  int Nano25E::SetCalibration(const string calibration_file_path) {
    DBGPRINT("%s", CALIBRATE_STR);
    cal_ = createCalibration(
      const_cast<char *>(calibration_file_path.c_str()), 1);
    if (cal_ == NULL) {
      printf("ERROR: Specified calibration could not be loaded\n");
      return ERROR_CALIBRATION;
    }
    SetForceUnits(cal_, "N");
    SetTorqueUnits(cal_, "N-m");
    return SUCCESS;
  }


  // Sets the voltages array with raw voltage values from the load cell
  int Nano25E::GetVoltages(float *voltages) const {
    if (task_handle_ == 0) {
      DBGPRINT("Load cell not started\n");
      return ERROR_DAQ_NOT_STARTED;
    } else {
      for (int i = 0; i < DOF; i++) {
        voltages[i] = voltages_[i];
      }
    }
    return SUCCESS;
  }
  int Nano25E::GetRawVoltages(float* voltages) const {
    if (task_handle_ == 0) {
      DBGPRINT("Load cell not started\n");
      return ERROR_DAQ_NOT_STARTED;
    }
    else {
      for (int i = 0; i < DOF; i++) {
        voltages[i] = raw_voltages_[i];
      }
    }
    return SUCCESS;
  }


  // Converts an array of voltages into forces and torques and
  // sets them into the parameter forces array.
  int Nano25E::GetLoads(float *forces) {
    if (task_handle_ == 0) {
      DBGPRINT("Load cell not started\n");
      return ERROR_DAQ_NOT_STARTED;
    } else {
      ConvertToFT(cal_, voltages_, forces);
    }
    return SUCCESS;
  }

  int Nano25E::GetRawLoads(float *forces) {
    if (task_handle_ == 0) {
      DBGPRINT("Load cell not started\n");
      return ERROR_DAQ_NOT_STARTED;
    }
    else {
      ConvertToFT(cal_, raw_voltages_, forces);
    }
    return SUCCESS;
  }


  // Performs a 6-axis translation/rotation on the transducer's coordinate
  // system in units mm and degrees.
  int Nano25E::SetTransformation(const float* transform) {
    DBGPRINT("%s", TRANSFORM_STR);
    unsigned int err_val = SetToolTransform(cal_,
      const_cast<float *>(transform), "mm", "degrees");
    if (err_val != SUCCESS) {
      return ERROR_TRANSFORMATION;
    }
    return SUCCESS;
  }


  // Bias based on the current voltage readings.
  int Nano25E::SetBias() {
    if (task_handle_ == 0) {
      DBGPRINT("Load cell not started\n");
      return ERROR_DAQ_NOT_STARTED;
    } else {
      ConvertToFT(cal_, raw_voltages_, bias_);  // sets bias_
      Bias(cal_, raw_voltages_);
      DBGPRINT("Set bias: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
        bias_[0], bias_[1], bias_[2], bias_[3], bias_[4], bias_[5]);
    }
    
    return SUCCESS;
  }

  // checks the bias set flag and sets input parameter bias if set
  int Nano25E::GetBias(float* bias) const {
    if (!bias_set_flag) {
      return ERROR_BIAS_NOT_SET;
    }
    for (int i = 0; i < DOF; ++i) {
      bias[i] = bias_[i];
    }
    return SUCCESS;
  }


  // Sets the absolute lower force threshold to eliminate load cell jitter.
  // ~50% of the system resolution can be consider jitter so set these values
  // accordingly.
  void Nano25E::SetLowerForceThreshold(
    const float* lower_threshold) {
    lower_threshold_set_flag = true;
    for (int i = 0; i < DOF; ++i) {
      lower_threshold_[i] = lower_threshold[i];
    }
  }


  // Returns if any of the transformed force/torque readings exceeds the
  // lower force threshold. Requires a lower force threshold be first set.
  bool Nano25E::ExceedsLowerForceThreshold() {
    if (!lower_threshold_set_flag) {
      fprintf(stderr, "Lower threshold has not been set\n");
    }
    if (task_handle_ == 0) {
      fprintf(stderr, "Load cell not started\n");
    } else {
      float loads[DOF];
      GetLoads(loads);
      for (int i = 0; i < DOF; ++i) {
        if (abs(loads[i]) > abs(lower_threshold_[i])) {
          return true;
        }
      }
    }
    return false;
  }

  // Checks task handle for running load cell task
  bool Nano25E::IsActive() {
    return task_handle_ != 0;
  }

  // Sets rate to sample_rate_ which was set during initialization
  void Nano25E::GetSamplingRate(float& rate) const {
    rate = sample_rate_;
  }

}  // namespace NanoLoadCell
