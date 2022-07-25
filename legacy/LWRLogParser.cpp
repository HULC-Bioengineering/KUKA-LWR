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

// LWRLogParser.cpp parses KUKA logs
#include "LWRLogParser.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/split.hpp>

#include <fstream>
#include <map>

using std::ifstream;
using std::string;
using std::vector;
using std::map;
using std::pair;

LWRLogParser::LWRLogParser(string const input_file_name)
    : input_file_name_(input_file_name), header_lines_(0), entries_(0), name_(""), state_(NEW) {}

LWRLogParser::LWRLogParser()
  : input_file_name_(""), header_lines_(0), entries_(0), name_(""), state_(EMPTY) {}

void LWRLogParser::SetLogFile(string const input_file_name) {
  input_file_name_ = input_file_name;
}

int LWRLogParser::ReadStandardizedCharacterPath(std::ifstream* char_file, vector<vector<float>>& char_path) {
  if (!char_file->is_open()) {
    printf("Unable to open input file");
    return Errors::ERROR_INVALID_INPUT_FILE;
  }

  string line;
  vector<string> line_split_values;

  // read in number of header lines (not including current line)
  getline(*char_file, line);
  boost::algorithm::split(line_split_values, line, boost::is_any_of("\t"),
    boost::token_compress_on);
  int header_lines = atoi(line_split_values.at(1).c_str());

  // ignore header lines
  int entries = 0;
  for (int i = 0; i <= header_lines; ++i) {
    getline(*char_file, line);
  }

  // push data into char_path
  float x;
  float y;
  float z;
  while (*char_file >> x >> y >> z) {
    char_path.push_back({x,y,z});
  }
  return Errors::SUCCESS;
}

int LWRLogParser::GetHeaderSize() {
  return header_lines_;
}

string LWRLogParser::GetHeaderName() {
  return name_;
}

string LWRLogParser::GetDate() {
  return date_;
}

void LWRLogParser::GetColumnFields(vector<string>& column_fields) {
  column_fields = column_fields_;
}

int LWRLogParser::GetEntries() {
  return entries_;
}

int LWRLogParser::ParseHeader() {
  if (state_ == EMPTY) {
    printf("ERROR, log file not set\n");
    return Errors::ERROR_NO_FILE_SET;
  }

  // Open KUKA log file
  ifstream input_file(input_file_name_);
  if (!input_file.is_open()) {
    printf("Unable to open input file");
    return Errors::ERROR_INVALID_INPUT_FILE;
  }

  vector<string> line_split_values;
  string line;
  string header_item;
  int start;
  int end;
  int element_num;

  // read in number of header lines (not including current line)
  getline(input_file, line);
  boost::algorithm::split(line_split_values, line, boost::is_any_of("\t"),
    boost::token_compress_on);
  header_lines_ = atoi(line_split_values.at(1).c_str());

  // unwrap the header goodness
  for (int i = 0; i < header_lines_; ++i) {
    getline(input_file, line);

    // split line on tab
    boost::algorithm::split(line_split_values, line, boost::is_any_of("\t"),
      boost::token_compress_on);
    header_item = line_split_values.at(0);
    header_item.pop_back();  // remove : from ending

    try {
      // switch on header meta data enum
      switch (header_meta_data.at(header_item)) {
      case NAME:
        name_ = line_split_values.at(1);
        break;
      case LOG_DATE:
        date_ = line_split_values.at(1);
        break;
      case SAMPLE_HZ:
        sample_rate_ = atoi(line_split_values.at(1).c_str());
        break;
      case COORDINATE_SYSTEM:
        coordinate_system_ = line_split_values.at(1);
        break;
      case CARTESIAN_ROTATION_MATRIX:
        for (int i = 0; i < NUMBER_OF_ROTATION_ELEMENTS; ++i) {
          cart_rotation_matrix_[i] = static_cast<float>(atof(line_split_values.at(i + 1).c_str()));
        }
        break;
      case PICTURE_DIMENSIONS:
        picture_width_ = atoi(line_split_values.at(1).c_str());
        picture_height_ = atoi(line_split_values.at(2).c_str());
        break;
      case CHARACTER_DIMENSIONS:
        character_width_ = atoi(line_split_values.at(1).c_str());
        character_height_ = atoi(line_split_values.at(2).c_str());
        break;
      case COLUMN_FIELDS:
        for (vector<string>::iterator it = line_split_values.begin() + 1;
          it != line_split_values.end(); ++it) {
          column_fields_.push_back(*it);
        }
        break;
      case COLUMN_ELEMENT_SIZE:
        start = 0;
        end = 0;
        element_num = 0;
        for (vector<string>::iterator it = line_split_values.begin() + 1; it != line_split_values.end(); ++it) {
          end = start + atoi(it->c_str());
          map_fieldname_elements_[column_fields_.at(element_num++)] = std::make_pair(start, end);
          start = end;
        }
        break;
      case ENTRIES:
        entries_ = atoi(line_split_values.at(1).c_str());
        break;
      }
    }
    catch (std::out_of_range) {
      printf("Unknown Header Information: %s\n", header_item.c_str());
    }
  }
  input_file.close();
  return Errors::SUCCESS;
}

void LWRLogParser::GetCharacterWidth(int* char_width) {
  *char_width = character_width_;
}
void LWRLogParser::GetCharacterHeight(int* char_height) {
  *char_height = character_height_;
}
void LWRLogParser::GetPoseRotationMatrix(float(&rotation_matrix)[NUMBER_OF_ROTATION_ELEMENTS]) {
  for (int i = 0; i < NUMBER_OF_ROTATION_ELEMENTS; ++i) {
    rotation_matrix[i] = cart_rotation_matrix_[i];
  }
}


// assume ifstream open
// puts the input_file stream to point at the start of cartesian position data
// does not close ifstream
int LWRLogParser::ParseHeader(ifstream* input_file, int* char_width, int* char_height, float(&pose_matrix)[NUMBER_OF_ROTATION_ELEMENTS]) const {

  vector<string> line_split_values;
  string line;
  string header_item;

  // read in number of header lines (not including current line)
  getline(*input_file, line);
  boost::algorithm::split(line_split_values, line, boost::is_any_of("\t"),
    boost::token_compress_on);
  int header_lines = atoi(line_split_values.at(1).c_str());

  // unwrap the header goodness
  for (int i = 0; i < header_lines - 1; ++i) {
    getline(*input_file, line);

    // split line on tab
    boost::algorithm::split(line_split_values, line, boost::is_any_of("\t"),
      boost::token_compress_on);
    header_item = line_split_values.at(0);

    try {
      // switch on header meta data enum
      switch (header_meta_data.at(header_item)) {
      case CARTESIAN_ROTATION_MATRIX:
        for (int j = 1; j <= NUMBER_OF_ROTATION_ELEMENTS; ++j) {
          pose_matrix[j-1] = static_cast<float>(atof(line_split_values.at(j).c_str()));
        }
        break;
      case CHARACTER_DIMENSIONS:
        *char_width = atoi(line_split_values.at(1).c_str());
        *char_height = atoi(line_split_values.at(2).c_str());
        break;
      }
    }
    catch (std::out_of_range) {
      printf("Unknown Header Information: %s", header_item.c_str());
    }
  }
  return Errors::SUCCESS;
}


int LWRLogParser::GetStartingJointPosition(std::vector<float>& starting_joint_vals) {
  vector<string> line_split_values;
  string line;
  
  if (state_ == EMPTY) {
    printf("ERROR, log file not set\n");
    return Errors::ERROR_NO_FILE_SET;
  }

  if (state_ == NEW) {
    ParseHeader();
  }

  // Open KUKA log file
  ifstream input_file(input_file_name_);
  if (!input_file.is_open()) {
    printf("Unable to open input file");
    return Errors::ERROR_INVALID_INPUT_FILE;
  }

  // ignore header lines
  for (int i = 0; i <= header_lines_; ++i) {
    getline(input_file, line);
  }

  getline(input_file, line);
  boost::algorithm::split(line_split_values, line, boost::is_any_of("\t "),
    boost::token_compress_on);

  for (map<string, pair<int, int>>::const_iterator iterator = map_fieldname_elements_.begin();
    iterator != map_fieldname_elements_.end(); iterator++) {
    int start = iterator->second.first;
    int end = iterator->second.second;
    int field_pos = 0;

    if (iterator->first.compare(COLUMN_FIELD_JOINT_POSITION) == 0) {
      for (int i = start; i < end; ++i) {
        starting_joint_vals.push_back(static_cast<float>(stof(line_split_values[i])));
      }
    }
  }
  return Errors::SUCCESS;
}

//Header Size : 10
//Name : STANDARDIZED $
//Date : Wed Feb 25 13 : 06 : 39 2015
//Coordinate System : Global Coordinate System
//Character Dimensions : 3	6
//TCP Rotation Matrix : 1	1	1	1	1	1	1	1	1
//Column Fields : CartesianPosition
//Column Element Size : 3
//Entries : 13860
// starts on top left corner of square which is 0,0
int LWRLogParser::ConvertMotionRecordLogToStandardizedCharLog() {
  int header_size = 8;

  ParseHeader();

  std::ofstream output(input_file_name_ + "STANDARDIZED");
  output << "Header Size:\t" << header_size << "\n";
  output << "Name:\t" << "STANDARDIZED " << GetHeaderName().c_str() << "\n";
  output << "Date:\t" << GetDate().c_str() << "\n";
  output << "Coordinate System:\t" << "Global Coordinate System" << "\n";
  output << "Character Dimensions:\t" << "3" << "\t" << "6" << "\n";
  output << "TCP Rotation Matrix:\t" << "0\t0\t-1\t0\t1\t0\t1\t0\t0" << "\n";
  output << "Column Fields:\t" << "CartesianPosition" << "\n";
  output << "Column Element Size:\t" << "3" << "\n";

  // loop over cart_path and output only displacement elements 3,7,11
  cart_motion cart_path;
  ParseMotion(NULL, &cart_path);

  // get first element for "zero point"
  float start_x = cart_path[0][3];
  float start_y = cart_path[0][7];
  float start_z = cart_path[0][11];

  // store buffer size 1 to compare current value against.
  // if no x,y,z movement don't include (who cares about rotation)
  float prev_position[3] = { start_x, start_y, start_z };
  int entries_count = 0;
  vector<vector<float>> out;
  out.reserve(20000);
  out.push_back({ 0, 0, 0 });

  for (auto pose : cart_path) {
    if (abs(pose[3] - prev_position[0]) > 0.00001 ||
      abs(pose[7] - prev_position[1]) > 0.00001 ||
      abs(pose[11] - prev_position[2]) > 0.00001) {

      // set prev position to this position

      prev_position[0] = pose[3];
      prev_position[1] = pose[7];
      prev_position[2] = pose[11];

      // output value
      out.push_back({ pose[3] - start_x, pose[7] - start_y, pose[11] - start_z });

      entries_count++;
    }
  }

  output << "Entries:\t" << entries_count << "\n";
  for (unsigned int i = 0; i < out.size(); i++) {
    output << std::fixed << std::setprecision(6) << out[i][0] << "\t" << out[i][1] << "\t" << out[i][2] << "\n";
  }

  return 1;
}


int LWRLogParser::ParseMotion(joint_motion* joint_pos_motion, cart_motion* cart_pose_motion) {
  vector<string> line_split_values;
  string line;

  // parse the header
  if (state_ == EMPTY) {
    printf("ERROR, log file not set\n");
    return Errors::ERROR_NO_FILE_SET;
  }
  else if (state_ == NEW) {
    ParseHeader();
  }
  
  // Open KUKA log file
  ifstream input_file(input_file_name_);
  if (!input_file.is_open()) {
    printf("Unable to open input file");
    return Errors::ERROR_INVALID_INPUT_FILE;
  }

  // ignore header lines
  for (int i = 0; i <= header_lines_; ++i) {
    getline(input_file, line);
  }

  // clear and reserve space for the motion path
  if (joint_pos_motion != NULL) {
    joint_pos_motion->clear();
    joint_pos_motion->reserve(entries_);
  }
  if (cart_pose_motion != NULL) {
    cart_pose_motion->clear();
    cart_pose_motion->reserve(entries_);
  }

  // parse the log
  int start = 0;
  int end = 0;
  int field_pos = 0;
  while (getline(input_file, line)) {
    boost::algorithm::split(line_split_values, line, boost::is_any_of("\t,"),
      boost::token_compress_on);

    for (map<string, pair<int, int>>::const_iterator iterator = map_fieldname_elements_.begin();
      iterator != map_fieldname_elements_.end(); iterator++) {
      start = iterator->second.first;
      end = iterator->second.second;
      field_pos = 0;

      if (iterator->first.compare(COLUMN_FIELD_JOINT_POSITION) == 0) {
        if (joint_pos_motion != NULL) {
          joint_pos_motion->push_back(boost::array<float, NUMBER_OF_JOINTS>());
          for (int i = start; i < end; ++i) {
            joint_pos_motion->back()[field_pos] = static_cast<float>(stof(line_split_values[i]));
            field_pos++;
          }
        }
      }
      else if (iterator->first.compare(COLUMN_FIELD_CARTESIAN_POSE) == 0) {
        if (cart_pose_motion != NULL) {
          cart_pose_motion->push_back(boost::array<float, NUMBER_OF_FRAME_ELEMENTS>());
          for (int i = start; i < end; ++i) {
            cart_pose_motion->back()[field_pos] = static_cast<float>(stof(line_split_values[i]));
            field_pos++;
          }
        }
      }
    }
  }
  input_file.close();
  return Errors::SUCCESS;
}
