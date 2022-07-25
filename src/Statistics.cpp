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

#include "Statistics.h"

namespace Stats {
  using std::unordered_map;
  using std::vector;
  using std::string;
  using std::ifstream;
  using std::ofstream;
  using std::fixed;
  using boost::accumulators::accumulator_set;
  using boost::accumulators::features;
  using Eigen::Vector3d;
  using namespace boost::accumulators;
  using boost::algorithm::split;

  // Get Fx, Fy, Fz, Mx, My, Mz from logged force file
  int GetLoadsFromCSV(string loads_file_path, vector<vector<double>>& loads) {
    std::ifstream loads_input_file(loads_file_path);
    if (!loads_input_file.is_open()) {
      printf("Unable to open forces input file");
      return ERROR_INVALID_INPUT_FILE;
    }

    string line;
    unsigned int count = 0;
    vector<string> line_split_loads;
    while (std::getline(loads_input_file, line)) {
      boost::algorithm::split(line_split_loads, line, boost::is_any_of(","),
        boost::token_compress_on);

      if (line_split_loads.size() == 1) {
        break;
      }
      loads.push_back({
        static_cast<double>(count++),  // count
        stof(line_split_loads[1]),     // fx
        stof(line_split_loads[2]),     // fy
        stof(line_split_loads[3]),     // fz
        stof(line_split_loads[4]),     // mx
        stof(line_split_loads[5]),     // my
        stof(line_split_loads[6]),     // mz
      });
    }
    return SUCCESS;
  }

  int LogAvgStdevMaxdev(string output_file, vector<vector<double>>& avg,
    vector<vector<double>>& stdev, vector<vector<double>>& maxdev) {

    std::ofstream output_stream;
    output_stream.open(output_file);
    output_stream << "Header Size:\t" << 4 << "\n";
    output_stream << "Name:\tStats for Nerds\n";
    output_stream << "Column Fields:\t" << "Item\t" << "Forces\t" << "Moments" << "\n";
    output_stream << "Column Element Size:\t" << "1\t" << "3\t" << "3" << "\n";
    output_stream << "Stat:\t" << "Mean" << "\n";
    for (vector<vector<double>>::iterator it = avg.begin(); it != avg.end(); ++it) {
      output_stream << it->at(0) << "\t" << it->at(1) << "\t" << it->at(2) << "\t" << it->at(3) << "\t" << it->at(4) << "\t" << it->at(5) << "\t" << it->at(6) << "\n";
    }
    output_stream << "Stat:\t" << "Standard Deviation" << "\n";
    for (vector<vector<double>>::iterator it = stdev.begin(); it != stdev.end(); ++it) {
      output_stream << it->at(0) << "\t" << it->at(1) << "\t" << it->at(2) << "\t" << it->at(3) << "\t" << it->at(4) << "\t" << it->at(5) << "\t" << it->at(6) << "\n";
    }
    output_stream << "Stat:\t" << "Max Deviation" << "\n";
    for (vector<vector<double>>::iterator it = maxdev.begin(); it != maxdev.end(); ++it) {
      output_stream << it->at(0) << "\t" << it->at(1) << "\t" << it->at(2) << "\t" << it->at(3) << "\t" << it->at(4) << "\t" << it->at(5) << "\t" << it->at(6) << "\n";
    }
    output_stream.close();
    return SUCCESS;
  }

  int GenerateNavByBendingPositionResults(vector<vector<float>>& msr_pose,
    vector<vector<double>>& dsr_position, string output_filename,
    Vector3d& avgs, Vector3d& stdevs) {

    ofstream os(output_filename);
    unsigned int count = 0;
    while (!os.is_open()) {
      printf("Unable to open output file. Prepending int.\n");
      os.open(std::to_string(count++) + output_filename);
    }

    // header stuff
    os << "Results standardized DSR @ point0=(0,0,0). Units are in (mm).\n";
    os << "Point, MSR X, DSR X, ABS DIFF X, MSR Y, DSR Y, ABS DIFF Y,\
           MSR Z, DES Z, ABS DIFF Z\n";

    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance >> x;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance >> y;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance >> z;

    for (unsigned int i = 0; i < dsr_position.size(); ++i) {
      os << i << ",";

      x(abs(msr_pose[i][3] - dsr_position[i][1]) * 1000);
      os << msr_pose[i][3] * 1000 << "," << dsr_position[i][1] * 1000 << "," <<
        abs(msr_pose[i][3] - dsr_position[i][1]) * 1000 << ",";

      y(abs(msr_pose[i][7] - dsr_position[i][2]) * 1000);
      os << msr_pose[i][7] * 1000 << "," << dsr_position[i][2] * 1000 << "," <<
        abs(msr_pose[i][7] - dsr_position[i][2]) * 1000 << ",";

      z(abs(msr_pose[i][11] - dsr_position[i][3]) * 1000);
      os << msr_pose[i][11] * 1000 << "," << dsr_position[i][3] * 1000 << "," <<
        abs(msr_pose[i][11] - dsr_position[i][3]) * 1000 << "\n";
    }

    avgs[0] = mean(x); stdevs[0] = sqrt(variance(x));
    avgs[1] = mean(y); stdevs[1] = sqrt(variance(y));
    avgs[2] = mean(z); stdevs[2] = sqrt(variance(z));
    return SUCCESS;
  }

  int GenerateNavByBendingPositionResults(const vector<vector<float>>& msr_pose,
    const vector<vector<double>>& dsr_position, const string output_filename) {

      ofstream os(output_filename);
      unsigned int count = 0;
      while (!os.is_open()) {
        printf("Unable to open output file. Prepending int.\n");
        os.open(std::to_string(count++) + output_filename);
      }

      // header stuff
      os << "Results standardized DSR @ point0=(0,0,0). Units are in (mm).\n";
      os << "Point, MSR X, DSR X, ABS DIFF X, MSR Y, DSR Y, ABS DIFF Y,\
                       MSR Z, DES Z, ABS DIFF Z\n";

      accumulator_set<float,
        features<tag::min, tag::max, tag::mean, tag::variance >> x;
      accumulator_set<float,
        features<tag::min, tag::max, tag::mean, tag::variance >> y;
      accumulator_set<float,
        features<tag::min, tag::max, tag::mean, tag::variance >> z;

      for (unsigned int i = 0; i < msr_pose.size(); ++i) {
        os << i << ",";

        x(abs(msr_pose[i][3] - dsr_position[i][1]) * 1000);
        os << msr_pose[i][3] * 1000 << "," << dsr_position[i][1] * 1000 << "," <<
          abs(msr_pose[i][3] - dsr_position[i][1]) * 1000 << ",";

        y(abs(msr_pose[i][7] - dsr_position[i][2]) * 1000);
        os << msr_pose[i][7] * 1000 << "," << dsr_position[i][2] * 1000 << "," <<
          abs(msr_pose[i][7] - dsr_position[i][2]) * 1000 << ",";

        z(abs(msr_pose[i][11] - dsr_position[i][3]) * 1000);
        os << msr_pose[i][11] * 1000 << "," << dsr_position[i][3] * 1000 << "," <<
          abs(msr_pose[i][11] - dsr_position[i][3]) * 1000 << "\n";
      }
      return SUCCESS;
  }

  int GenerateNerdStatsFile(string input_filename, string input_file_extension,
    unsigned int number_trials, unsigned int points_per_trial) {
 
    unsigned int err_val = SUCCESS;
    string loads_file;
    string output_stats_file = input_filename + "NerdStats.csv";
    vector<vector<vector<double>>> all_trials_loads(number_trials);
    
    // get all trials loads
    for (unsigned int i = 0; i < number_trials; ++i) {
      loads_file = input_filename + std::to_string(i) + input_file_extension;
      err_val = GetLoadsFromCSV(loads_file, all_trials_loads[i]);
      if (err_val != SUCCESS) {
        return ERROR_CSV_LOADS;
      }
    }

    // calculate avg, stdev, maxdev
    vector<vector<double>> avg;
    vector<vector<double>> stdev;
    vector<vector<double>> maxdev;
    CalculateAvgStdevMaxdev(all_trials_loads, avg, stdev, maxdev);

    // write to output file
    LogAvgStdevMaxdev(output_stats_file, avg, stdev, maxdev);
    return SUCCESS;
  }

  void CalculateAvgStdevMaxdev(
    vector<vector<vector<double>>>& all_trials,
    vector<vector<double>>& avg,
    vector<vector<double>>& stdev,
    vector<vector<double>>& maxdev) {

    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance>> tmp;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance>> x;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance>> y;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance>> z;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance>> mx;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance>> my;
    accumulator_set<float,
      features<tag::min, tag::max, tag::mean, tag::variance>> mz;

    int entries = all_trials[0].size();  // assume all trials have same #points
    for (int i = 0; i < entries; ++i) {
      for (unsigned int j = 0; j < all_trials.size(); ++j) {
        x(all_trials[j][i][1]);
        y(all_trials[j][i][2]);
        z(all_trials[j][i][3]);
        mx(all_trials[j][i][4]);
        my(all_trials[j][i][5]);
        mz(all_trials[j][i][6]);
      }
      avg.push_back({ all_trials[0][i][0], mean(x), mean(y), mean(z), mean(mx), mean(my), mean(mz) });
      stdev.push_back({ all_trials[0][i][0], sqrt(variance(x)), sqrt(variance(y)), sqrt(variance(z)), sqrt(variance(mx)), sqrt(variance(my)), sqrt(variance(mz)) });
      maxdev.push_back({ all_trials[0][i][0], MaxDev(x), MaxDev(y), MaxDev(z), MaxDev(mx), MaxDev(my), MaxDev(mz) });

      // reset accumulator(s)
      x = y = z = mx = my = mz = tmp;
    }
  }

  float MaxDev(accumulator_set<float, features<tag::min, tag::max, tag::mean, tag::variance>> &acc) {
    return std::max(max(acc) - mean(acc), mean(acc) - min(acc));
  }

}  // end namespace