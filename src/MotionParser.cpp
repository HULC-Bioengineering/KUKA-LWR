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
#include "MotionParser.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/split.hpp>

#include <fstream>
#include <map>

namespace MotionParsing {
  using std::ifstream;
  using std::ofstream;
  using std::string;
  using std::vector;
  using std::map;
  using std::pair;
  using boost::algorithm::split;
  using boost::is_any_of;
  using boost::token_compress_on;
  using Eigen::Vector3d;
  using Eigen::Matrix3f;
  using namespace HeaderInfo;

  namespace HeaderInfo {

    Header::Header() : header_size_(0), name_(""), date_(""),
      coordinate_system_(""), sample_rate_(0), entries_(0), picture_width_(0),
      picture_height_(0), character_width_(0), character_height_(0),
      cart_rotation_matrix_()
    {}

    // if properly configured header first line should contain:
    // Header Size:  NUM_LINES
    // get number of header lines
    int Header::GetHeaderSize(ifstream* if_stream) {
      vector<string> line_split_values;
      string line;
      int header_size = 0;
      getline(*if_stream, line);
      if (line.find(header_meta_data.right.find(HEADER_SIZE)->second) != std::string::npos) {
        split(line_split_values, line, is_any_of("\t"), token_compress_on);
        header_size = atoi(line_split_values.at(1).c_str());
      }
      else {
        printf("Header not configured");
      }
      return header_size;
    }

    int Header::Parse(ifstream* if_stream) {
      if (if_stream == NULL) {
        return ERROR_FILE_DOES_NOT_EXIST;
      }
      if (!if_stream->is_open()) {
        printf("Input file not opened");
        return ERROR_FILE_DOES_NOT_EXIST;
      }

      // make a copy of the pointer
      ifstream* if_stream_cpy = if_stream;

      vector<string> line_split_values;
      string line;
      string header_item;

      int start;
      int end;
      int element_num;

      header_size_ = GetHeaderSize(if_stream_cpy);
      // if 0 then header does not follow proper format
      if (header_size_ == 0) {
        proper_ = false;
      }
      else {
        proper_ = true;
      }

      // unwrap the header goodness
      for (int i = 0; i < header_size_; ++i) {
        getline(*if_stream_cpy, line);

        split(line_split_values, line, is_any_of("\t"), token_compress_on);
        header_item = line_split_values.at(0);
        header_item.pop_back();  // remove : from ending

        try {
          // switch on header meta data enum
          switch (header_meta_data.left.find(header_item)->second) {
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
      return SUCCESS;
    }

    int Header::GetHeaderSize() {
      return header_size_;
    }

    string Header::GetHeaderName() {
      return name_;
    }

    string Header::GetDate() {
      return date_;
    }

    void Header::GetColumnFields(vector<string>& column_fields) {
      column_fields = column_fields_;
    }

    int Header::GetEntries() {
      return entries_;
    }

    void Header::GetCharacterWidth(int& char_width) {
      char_width = character_width_;
    }
    void Header::GetCharacterHeight(int& char_height) {
      char_height = character_height_;
    }
    void Header::GetPoseRotationMatrix(float(&rotation_matrix)[NUMBER_OF_ROTATION_ELEMENTS]) {
      for (int i = 0; i < NUMBER_OF_ROTATION_ELEMENTS; ++i) {
        rotation_matrix[i] = cart_rotation_matrix_[i];
      }
    }
    void Header::WriteHeader(std::ofstream& os, HEADER_META_DATA val_type, string val) {
      switch (val_type) {
      case HEADER_SIZE:
        break;
      case NAME:
        break;
      case LOG_DATE:
        break;
      case SAMPLE_HZ:
        break;
      case COORDINATE_SYSTEM:
        break;
      case CARTESIAN_ROTATION_MATRIX:
        break;
      case PICTURE_DIMENSIONS:
        break;
      case CHARACTER_DIMENSIONS:
        break;
      case COLUMN_FIELDS:
        break;
      case COLUMN_ELEMENT_SIZE:
        break;
      case ENTRIES:
        break;
      }
    }

    bool Header::IsProperHeader() {
      return proper_;
    }

    void Header::WriteDbgHeader(std::ofstream& os) {
      os << DBG_HEADER;
    }


  }  // end header namespace

  MotionParser::MotionParser(const string input_filename, const string output_filename) :
    input_filename_(input_filename), output_filename_(output_filename) {
    if_stream_.open(input_filename);
    if (!if_stream_.is_open()) {
      printf("Input file %s does not exist", input_filename);
    }
    of_stream_.open(output_filename);
    if (!of_stream_.is_open()) {
      printf("Output file %s does not exist", output_filename);
    }
  }

  MotionParser::MotionParser() : input_filename_(""), output_filename_("") {}

  int MotionParser::Open() {
    OpenInput();
    OpenOutput();
  }

  int MotionParser::OpenInput() {
    if (input_filename_ != "") {
      if_stream_.open(input_filename_);
      if (!if_stream_.is_open()) {
        printf("Input file %s does not exist", input_filename_);
        return ERROR_FILE_DOES_NOT_EXIST;
      }
    }
    else {
      printf("Input file %s not set", input_filename_);
    }
    return SUCCESS;
  }

  int MotionParser::OpenOutput() {
    if (output_filename_ != "") {
      of_stream_.open(output_filename_);
      if (!of_stream_.is_open()) {
        printf("Output file %s does not exist", output_filename_);
        return ERROR_FILE_DOES_NOT_EXIST;
      }
    }
    else {
      printf("Output file %s not set", output_filename_);
    }
    return SUCCESS;
  }

  int MotionParser::SetInputFile(const string input_filename) {
    input_filename_ = input_filename;
    return OpenInput();
  }

  int MotionParser::SetOutputFile(const string output_filename) {
    output_filename_ = output_filename;
    return OpenOutput();
  }

  void MotionParser::GetCharacterWidth(int& char_width) {
    header_.GetCharacterWidth(char_width);
  }
  void MotionParser::GetCharacterHeight(int& char_height) {
    header_.GetCharacterHeight(char_height);
  }
  void MotionParser::GetPoseRotationMatrix(float(&rotation_matrix)[NUMBER_OF_ROTATION_ELEMENTS]) {
    header_.GetPoseRotationMatrix(rotation_matrix);
  }

  int MotionParser::ReadHeader() {
    return header_.Parse(&if_stream_);
  }

  int MotionParser::ParseMotion(vector<vector<float>>* joint_path, vector<vector<float>>* cart_path) {
    string line;
    vector<string> line_split_values;

    // parse the header
    header_.Parse(&if_stream_);
    //joint_path->reserve(header_.GetEntries());

    // parse the log
    while (getline(if_stream_, line)) {
      boost::algorithm::split(line_split_values, line, boost::is_any_of("\t,"),
        boost::token_compress_on);

      for (map<string, pair<int, int>>::const_iterator iterator = header_.map_fieldname_elements_.begin();
        iterator != header_.map_fieldname_elements_.end(); iterator++) {

        if (iterator->first.compare(COLUMN_FIELD_JOINT_POSITION) == 0) {
          if (joint_path != NULL) {
            vector<float> v;
            for (int i = iterator->second.first; i < iterator->second.second; ++i) {
              v.push_back(static_cast<float>(stof(line_split_values[i])));
            }
            joint_path->push_back(v);
          }
        }
        else if (iterator->first.compare(COLUMN_FIELD_CARTESIAN_POSE) == 0) {
          if (cart_path != NULL) {
            vector<float> v;
            for (int i = iterator->second.first; i < iterator->second.second; ++i) {
              v.push_back(static_cast<float>(stof(line_split_values[i])));
            }
            cart_path->push_back(v);
          }
        }
      }
    }
    return SUCCESS;
  }

} // end namespace
