#include <string>
#include <vector>
#include <fstream>
#include <utility>
#include "Config.h"
#include "Utils.h"
#include "LWR.h"
#include "Statistics.h"
#include "Nano25E.h"
#include "MotionParser.h"
#include <WindowsAbstraction.h>

using std::string;
using std::vector;
using std::ofstream;
using std::pair;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using NanoLoadCell::Nano25E;
using MotionParsing::MotionParser;
using Stats::GenerateNerdStatsFile;

// Collaboration mode where the LWR is logging joints, cartesian pose,
// and timestamp.
// Parameters:
//   lwr: KUKA controller.
//   log_file_name: desired name of the log file. Note: log path is set in
//     the FRI Library calibration file.
//   mode: joint positioning or cartesian impedance control
//   stop_condition: stop logging based on: joint position, time, esc
int RecordMotion(LWR& lwr, std::string log_file_name, ControlMode mode,
    RecordStopCondition stop_condition);

// Replay motion from file in KUKA log directory given by log_file_name.
// Parameters:
//   lwr: KUKA controller.
//   log_file_name: name of the log file to replay.
//   mode: joint positioning or cartesian impedance control
int ReplayMotion(LWR& lwr, std::string log_file_name, ControlMode mode);

// Follow/replay a joint motion path and record load cell values transformed
// to tool coordinate system at each point. Stop for 500ms once point in
// found before recording load cell values to ignore dynamic effects.
// Parameters:
//   lwr: KUKA controller.
//   lc: Nano25E load cell.
//   follow_path_filename: full file name for joint path.
//   output_loads_filename: full file name for the joint and pose log.
//   mode: joint position or cartesian impedance. Note: currently only joint
//         position is functional. #TODO cartesian impedance.
int FollowPath(LWR& lwr, Nano25E& lc, std::string follow_path_filename,
  std::string output_loads_filename, ControlMode mode);

// Cartesian Impedance Control Mode logging pose and load cell values
// transformed to the tool frame. Intended to move the LWR by hand to
// different positions while loads and poses are logged.
// Parameters:
//   lwr: KUKA controller.
//   lc: Nano25E load cell.
//   output_file: load cell and pose logging file.
//   restrictRotation: inhibits rotational changes from the starting pose.
int FreeMovementPoseLoadRecord(LWR& lwr, Nano25E& lc, std::string output_file,
  bool restrictRotation = true);

// Gravity compensation collaborative cartesian impedance control mode.
// Used to move the robot into pose by hand. Sets stiffness low and updates
// pose changes based on previous measured pose.
// Parameters:
//   lwr: KUKA controller.
//   duration: time in seconds which free movement is allowed
int FreeCartesianMovement(LWR& lwr, int duration);

// Main algorithm as described in thesis Chapter 4. Move to a pose which
// satasfies a bending loads point. Once a point is found, record pose.
// Uses tool directional components to reduce set point threshold satisfaction
// errors.
// Parameters:
//   lwr: KUKA controller.
//   lc: Nano25E load cell.
//   follow_loads_path: bending path.
//   output_position_filename: log file for found points poses.
//   point_delay: delay between points
int NavByBendingForces(LWR& lwr, Nano25E& lc,
  const std::string follow_loads_path, const std::string output_filename,
  const unsigned int point_delay=0);

// The place where you mess around trying your functions out
// Parameters:
//   lwr: KUKA controller.
//   lc: Nano25E load cell.
void Debugging(LWR& lwr, Nano25E& lc);

// The place where you mess around trying your functions out
// Parameters:
//   lwr: KUKA controller.
//   lc: Nano25E load cell.
//   response: Response to prompted question.
void FirstScript (string response);
