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
#ifndef STATISTICS_H_
#define STATISTICS_H_

#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\accumulators\accumulators.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\accumulators\statistics\max.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\accumulators\statistics\min.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\accumulators\statistics\mean.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\accumulators\statistics\stats.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\accumulators\statistics\variance.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\algorithm\string.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\algorithm\string\join.hpp>
#include <C:\Users\HMMS\Documents\GitHub\Thesis\KUKA LWR\external\boost_1_57_0\boost\algorithm\string\split.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Stats {

  const unsigned int SUCCESS = 0;
  const unsigned int ERROR_INVALID_INPUT_FILE = 1;
  const unsigned int ERROR_CSV_LOADS = 2;

  int GetLoadsFromCSV(std::string loads_file_path,
    std::vector<std::vector<double>>& loads);

  int LogAvgStdevMaxdev(std::string output_file,
    std::vector<std::vector<double>>& avg,
    std::vector<std::vector<double>>& stdev,
    std::vector<std::vector<double>>& maxdev);

  int GenerateNavByBendingPositionResults(
    std::vector<std::vector<float>>& msr_pose,
    std::vector<std::vector<double>>& dsr_pose,
    std::string output_filename, Eigen::Vector3d& avgs,
    Eigen::Vector3d& stdev);

  int GenerateNavByBendingPositionResults(
    const std::vector<std::vector<float>>& msr_pose,
    const std::vector<std::vector<double>>& dsr_pose,
    const std::string output_filename);

  int GenerateNerdStatsFile(std::string input_filename,
    std::string input_file_extension, unsigned int number_trials,
    unsigned int points_per_trial);

  void CalculateAvgStdevMaxdev(std::vector<std::vector<std::vector<double>>>& all_trials,
    std::vector<std::vector<double>>& avg, std::vector<std::vector<double>>& stdev,
    std::vector<std::vector<double>>& maxdev);

  float MaxDev(boost::accumulators::accumulator_set<float,
    boost::accumulators::features<boost::accumulators::tag::min,
    boost::accumulators::tag::max, boost::accumulators::tag::mean,
    boost::accumulators::tag::variance >> &acc);
  
};

#endif  // LOGGER_H_
