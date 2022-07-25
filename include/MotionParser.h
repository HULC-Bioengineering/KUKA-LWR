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

// I'm just going to appologize in advance. This is horrid. All this does is
// read a recorded log from a motion record. It's just awful but it works.
// #TODO (matthewbstokes) tlc.
#ifndef MOTIONPARSER_H_
#define MOTIONPARSER_H_

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <stdexcept>
#include <fstream>
#include <iomanip>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace MotionParsing {
  const unsigned int NUMBER_OF_ROTATION_ELEMENTS = 9;
  struct load {
    int t;
    float x;
    float y;
    float z;
    float mx;
    float my;
    float mz;
  };

  namespace HeaderInfo {
    const std::string COLUMN_FIELD_JOINT_POSITION = "JointPosition";
    const std::string COLUMN_FIELD_CARTESIAN_POSE = "CartesianPose";
    const std::string DBG_HEADER = 
      "Raw LC Fx, Raw LC Fy, Raw LC Fz, Raw LC Mx, Raw LC My, Raw LC Mz,\
      Dsr LC Fx, Dsr LC Fy, Dsr LC Fz, Dsr LC Mx, Dsr LC My, Dsr LC Mz,\
      Msr X, Msr Y, Msr Z,\
      Dsr X, Dsr Y, Dsr Z,\n";
    

    enum HEADER_META_DATA {
      HEADER_SIZE,
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

    static const boost::bimap<std::string, HEADER_META_DATA> header_meta_data =
      boost::assign::list_of<boost::bimap<std::string,
      HEADER_META_DATA>::value_type>
      (std::string("Header Size"), HEADER_SIZE)
      (std::string("Name"), NAME)
      (std::string("Date"), LOG_DATE)
      (std::string("Sample Rate"), SAMPLE_HZ)
      (std::string("Coordinate System"), COORDINATE_SYSTEM)
      (std::string("TCP Rotation Matrix"), CARTESIAN_ROTATION_MATRIX)
      (std::string("Picture Dimensions"), PICTURE_DIMENSIONS)
      (std::string("Character Dimensions"), CHARACTER_DIMENSIONS)
      (std::string("Column Fields"), COLUMN_FIELDS)
      (std::string("Column Element Size"), COLUMN_ELEMENT_SIZE)
      (std::string("Entries"), ENTRIES);

    class Header {
    private:
      std::string name_;
      std::string date_;
      std::string coordinate_system_;
      int header_size_;
      int sample_rate_;
      int entries_;
      int picture_width_;
      int picture_height_;
      int character_width_;
      int character_height_;
      bool proper_;
      float cart_rotation_matrix_[NUMBER_OF_ROTATION_ELEMENTS];     
      std::vector<std::string> column_fields_;

    public:
      std::map<std::string, std::pair<int, int>> map_fieldname_elements_;
      Header();
      int Parse(std::ifstream* if_stream);
      int GetHeaderSize(std::ifstream* if_stream);
      int GetHeaderSize();
      std::string GetHeaderName();
      std::string GetDate();
      void GetColumnFields(std::vector<std::string>& column_fields);
      int GetEntries();
      void GetCharacterWidth(int& char_width);
      void GetCharacterHeight(int& char_height);
      void GetPoseRotationMatrix(
        float(&rotation_matrix)[NUMBER_OF_ROTATION_ELEMENTS]);
      bool IsProperHeader();
      void WriteHeader(std::ofstream& os,
        HeaderInfo::HEADER_META_DATA meta_type, const std::string val);
      void WriteDbgHeader(std::ofstream& os);
    };
  }

  const unsigned int SUCCESS = 0;
  const unsigned int ERROR_FILE_DOES_NOT_EXIST = 1;
  const unsigned int ERROR_IMPROPER_HEADER = 2;

  class MotionParser {
  private:
    std::string input_filename_;
    std::ifstream if_stream_;
    std::string output_filename_;
    std::ofstream of_stream_;
    HeaderInfo::Header header_;

  public:
    MotionParser();
    MotionParser(const std::string input_filename,
      const std::string output_filename);
    int ReadHeader();
    int Open();
    int OpenInput();
    int OpenOutput();
    bool IsOutputSet();
    bool IsInputSet();
    int SetInputFile(const std::string input_filename);
    int SetOutputFile(const std::string output_filename);
    int ReadStandardizedCharacterPath(std::ifstream* char_file,
      std::vector<std::vector<float>>& char_path);
    void SetLogFile(std::string const input_file_name);
    int ParseMotion(std::vector<std::vector<float>>* joint_path,
      std::vector<std::vector<float>>* cart_path);
    int WriteDbgHeader();
    int WriteLoadsAndPositionsStream(std::vector<load>& loads,
      std::vector<load>& pos_rot);
    int WriteLoadsAndPositions(std::vector<load>& loads,
      std::vector<load>& pos_rot);
    void GetCharacterWidth(int& char_width);
    void GetCharacterHeight(int& char_height);
    void GetPoseRotationMatrix(
      float(&rotation_matrix)[NUMBER_OF_ROTATION_ELEMENTS]);

  };
}

#endif  // MOTIONPARSER_H_
