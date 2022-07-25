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

// LWRLogParser.cpp parses KUKA logs
#ifndef LWRLOGPARSER_H_
#define LWRLOGPARSER_H_

#include "Config.h"

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <stdexcept>
#include <iomanip>  // setprecision

enum LOG_STATE {EMPTY, NEW, PARSED};

enum HEADER_META_DATA {
  NAME,
  LOG_DATE,
  SAMPLE_HZ,
  COORDINATE_SYSTEM,
  CARTESIAN_ROTATION_MATRIX,
  PICTURE_DIMENSIONS,
  CHARACTER_DIMENSIONS,
  COLUMN_FIELDS,
  COLUMN_ELEMENT_SIZE,
  ENTRIES
};
enum COLUMN_FIELD {
  ITEM,
  TIMESTAMP,
  JOINT_POSITION,
  CARTESIAN_POSE,
  CARTESIAN_POSITION,
};

static std::unordered_map<std::string, HEADER_META_DATA> header_meta_data = {
  { "Name", NAME },
  { "Date", LOG_DATE },
  { "Sample Rate", SAMPLE_HZ },
  { "Coordinate System", COORDINATE_SYSTEM },
  { "TCP Rotation Matrix", CARTESIAN_ROTATION_MATRIX },
  { "Picture Dimensions", PICTURE_DIMENSIONS },
  { "Character Dimensions", CHARACTER_DIMENSIONS },
  { "Column Fields", COLUMN_FIELDS },
  { "Column Element Size", COLUMN_ELEMENT_SIZE },
  { "Entries", ENTRIES },
};

// Parse a KUKA log file for: motion (vector of joint angles).
class LWRLogParser {
  const std::string COLUMN_FIELD_JOINT_POSITION = "JointPosition";
  const std::string COLUMN_FIELD_CARTESIAN_POSE = "CartesianPose";

 private:
   // Name of the log file to parse
  std::string input_file_name_;
  int header_lines_;
  int entries_;
  std::string name_;
  LOG_STATE state_;
  std::string date_;
  std::map<std::string, std::pair<int, int>> map_fieldname_elements_;
  int sample_rate_;
  std::vector<std::string> column_fields_;
  std::string coordinate_system_;
  int picture_width_;
  int picture_height_;
  int character_width_;
  int character_height_;
  float cart_rotation_matrix_[NUMBER_OF_ROTATION_ELEMENTS];

 public:
  LWRLogParser();
  LWRLogParser(std::string const input_file_name);
  int ParseHeader();
  int ParseHeader(std::ifstream* input_file, int* char_width, int* char_height, float(&rotation_matrix)[NUMBER_OF_ROTATION_ELEMENTS]) const ;
  int GetStartingJointPosition(std::vector<float>& starting_joint_vals);
  void SetLogFile(std::string const input_file_name);
  int ParseMotion(joint_motion* joint_pos_motion=NULL,
    cart_motion* cartpose_motion=NULL);
  void GetCharacterWidth(int* char_width);
  void GetCharacterHeight(int* char_height);
  void GetPoseRotationMatrix(float(&rotation_matrix)[NUMBER_OF_ROTATION_ELEMENTS]);
  int ReadStandardizedCharacterPath(std::ifstream* char_file, std::vector<std::vector<float>>& char_path);
  int GetHeaderSize();
  void GetColumnFields(std::vector<std::string>& column_fields);
  std::string GetHeaderName();
  std::string GetDate();
  int GetEntries();
  int ConvertMotionRecordLogToStandardizedCharLog();
};
#endif  // C:_USERS_HMMS_FRI_FRILIBRARY_WINDOWS_KUKA_LWRLOGPARSER_H_
