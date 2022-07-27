#include "Scripts.h"
#include <chrono>
using namespace std::chrono;
///////////////////////////////////////////////////////////////////////////////
////                         NAV BY BENDING HELPERS                        ////
///////////////////////////////////////////////////////////////////////////////

// Apply the transformation from the measured load cell values to the current
// pose resulting in the next step / desired pose.
//
// Current position and orientation + change in position = next position + orientation
// where:
//   change_in_position == distance in mm (forces/torques already converted to mm)

// Parameters:
//   pose: Current Cartesian pose as as 1x12 array.
//   loads: Load cell converted force/torques (current step in mm)
//          values transformed to the tools coordinate system located
//          at the TCP and converted to position and rotational changes (via PID).
//   next_pose: pointer to an array which is set to T ti+1 wrt base.
static void ForceMovement(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
  const double(&loads)[LOAD_CELL_DOF], float* next_pose) {
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> result_pose_matrix;
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;
  
  // create eigen matrix structure from the current cartesian pose
  // 1x12 -> 4x4
  // base
  //     T
  //   ti
  current_pose_matrix <<
    pose[0], pose[1], pose[2], pose[3],
    pose[4], pose[5], pose[6], pose[7],
    pose[8], pose[9], pose[10], pose[11],
    0, 0, 0, 1;

  // create eigen matrix from load cell values using
  // fixed X-Y-Z rotation sequence. Load cell values should already be
  // transformed to the tools coordinate system and located at the TCP
  // ti        [1, 0, 0, loads[0]]
  //     T  -> [0, 1, 0, loads[1]]
  // ti+1      [0, 0, 1, loads[2]]
  //           [0, 0, 0,    1    ]
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
  float sA = static_cast<float>(sin(RAD(loads[3])));
  float sB = static_cast<float>(sin(RAD(loads[4])));
  float sG = static_cast<float>(sin(RAD(loads[5])));
  float cA = static_cast<float>(cos(RAD(loads[3])));
  float cB = static_cast<float>(cos(RAD(loads[4])));
  float cG = static_cast<float>(cos(RAD(loads[5])));
  lc_matrix <<
    cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), loads[0],
    sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), loads[1],
    -sB, cB*sG, cB*cG, loads[2],
    0, 0, 0, 1;

  // Find base, ti+1
  // base     ti       base
  //     T      T  =       T
  //   ti   ti+1       ti+1
  result_pose_matrix = current_pose_matrix * lc_matrix;

  // set next cartesian pose for FRI communication
  for (unsigned int i = 0; i < 12; ++i) {
    next_pose[i] = result_pose_matrix((i/4), (i%4));
  }
}

// Safety check if current pose is within the maxiumum bounds of the desired
// position. The check current uses positonal values and no rotations.
//
// Parameters:
//   dsr_position: Vector containing the desired [X, Y, Z] position.
//   msr_pose: Current measured pose.
//   max_bounds: Vector containing the maximum X, Y, Z positional offset.
//               Values outside this range are considered out of bounds.
//
// Returns:
//  true if all X, Y, Z values of the current measured pose are within the
//  maximum bounds, else false.
bool InBounds(const vector<double>& dsr_position, const float(&msr_pose)[12],
  const vector<double>& max_bounds) {
  if (abs(dsr_position[0] - msr_pose[3]) > max_bounds[0] ||
    abs(dsr_position[1] - msr_pose[7]) > max_bounds[1] ||
    abs(dsr_position[2] - msr_pose[11]) > max_bounds[2]) {
    DBGPRINT("Out of bounds differential: %f, %f, %f\n",
      abs(dsr_position[0] - msr_pose[3]),
      abs(dsr_position[1] - msr_pose[7]),
      abs(dsr_position[2] - msr_pose[11]));
    return false;
  }
  return true;
}

// Force path.
// Point1         ->    Point2
// [Fx, Fy, Fz]         [Fx, Fy, Fz]
// E.g. with values for an X travel path
//
// 
// Fx=1, Fy=1, Fz=1    ->  Fx=2, Fy=4, Fz=-1
// Deadbands (lower, upper limit):
//   Fx: [1,  2]
//   Fy: [1,  4]
//   Fz: [-1, 1]
// vector == self managing/exapanding list. Smart array which does not need fixed size.
// Args:
//   loads: A list of points for each point you have vector containing X,Y,Z.
//   deadbands: A list of points each point containing and lower and upper bounds for X, Y, Z.
// Returns:
//   For every point X, Y and Z each have an upper and lower deadband limit.
void CalculateDeadbands(const vector<vector<double>>& loads, vector<vector<pair<double, double>>>& deadbands) {
  for (unsigned int path_point = 0; path_point < loads.size() - 1; ++path_point) {
    vector<pair<double, double>> v;
    for (unsigned int i = 0; i < 3; ++i) {
      pair<double, double> p;

      if (loads[path_point][i] < loads[path_point + 1][i]) {
        p.first = loads[path_point][i];
        p.second = loads[path_point + 1][i];
      }
      else {
        p.first = loads[path_point + 1][i];
        p.second = loads[path_point][i];
      }
      DBGPRINT("Deadband point %d dir %d: %.5f, %.5f",
                path_point, i, p.first, p.second);
      v.push_back(p);
    }
    deadbands.push_back(v);
  }
}

// Check if the current value is between the deadband.
//
// Parameters:
//   val: Value to check.
//   range: deadband range (lower, upper).
// Returns:
//   true is value is within deadband range, else false.
bool InRange(double val, pair<double, double> range) {
	return val >= range.first && val <= range.second;
}

// Sets the current PID turning parameters. Each of the PID's tuning parameters
// can vary depending on travel direction giving extensive control.
// Gain scheduling where 
// Layout/Format. 
//   // X direction of travel
//     // PID_x
//        // P parameter
//        // I parameter
//        // D parameter
//     // PID_y
//        // P parameter
//        // I parameter
//        // D parameter
//     // PID_z
//        // P parameter
//        // I parameter
//        // D parameter
//   // Y direction of travel
//     // PID_x
//        // P parameter
//        // I parameter
//        // D parameter
//     // PID_y
//        // P parameter
//        // I parameter
//        // D parameter
//     // PID_z
//        // P parameter
//        // I parameter
//        // D parameter
//   // Z direction of travel
//     // PID_x
//        // P parameter
//        // I parameter
//        // D parameter
//     // PID_y
//        // P parameter
//        // I parameter
//        // D parameter
//     // PID_z
//        // P parameter
//        // I parameter
//        // D parameter
// 
// Parameters:
//   pid_tune: 3 dimensional matrix of tuning parameters containing.
//   direction: Direction of positional travel between the current path
//              point and next path point. Direction is a struct which has
//              both a value (X=0, Y=1, Z=2) and sign (+, -).
//   pids: Vector containing the X, Y, Z PID controllers.
void SetDirectionPIDTunings(const vector<vector<vector<double>>>& tuning,
  const Direction& direction, vector<PID>& pids) {

  switch (direction.axis) {
  case X:
    pids[0].setTunings(tuning[0][0][0], tuning[0][0][1], tuning[0][0][2]);
    pids[1].setTunings(tuning[0][1][0], tuning[0][1][1], tuning[0][1][2]);
    pids[2].setTunings(tuning[0][2][0], tuning[0][2][1], tuning[0][2][2]);
    break;
  case Y:
    pids[0].setTunings(tuning[1][0][0], tuning[1][0][1], tuning[1][0][2]);
    pids[1].setTunings(tuning[1][1][0], tuning[1][1][1], tuning[1][1][2]);
    pids[2].setTunings(tuning[1][2][0], tuning[1][2][1], tuning[1][2][2]);
    break;
  case Z: 
    pids[0].setTunings(tuning[2][0][0], tuning[2][0][1], tuning[2][0][2]);
    pids[1].setTunings(tuning[2][1][0], tuning[2][1][1], tuning[2][1][2]);
    pids[2].setTunings(tuning[2][2][0], tuning[2][2][1], tuning[2][2][2]);
    break;
  default:
    printf("Unknown PID direction of travel");
  }
}

// PID's output limits are used to limit the translation/rotation in a single
// iteration step (2ms) to an acceptable/controllable range. Once a path
// point is reached the next path point might be far away and thus without
// imposing a limit the controller would jerk/exceed rapidly accelerate
// and/or exceed plausable motion kernal limitations.
//
// Additionally, in the direction of travel, PID output limits are not set
// to limits centered around 0, but rather between a "slow" and "fast" step
// size thus travel will always occur in said direction. This is to be used
// in conjunction with a limit whereby the controller can never cause backward
// movement as when a reaming tool is attached material is assumed to have been
// removed in the backward direction.
//
// Parameters:
//   max_step: Maxiumum allowable step size.
//   lower_path: Lower step size limit in travel direction.
//   direction: Direction of positional travel between the current path
//              point and next path point. Direction is a struct which has
//              both a value (X=0, Y=1, Z=2) and sign (+, -).
//   pids: Vector containing the X, Y, Z PID controllers.
void SetDirectionPIDOutputs(const double max_step, const double lower_path,
  Direction& direction, vector<PID>& pids) {

  // solve the sign issue after
  // also don't play with the gains sign
  switch (direction.axis) {
  case X:
    pids[0].setOutputLimits(-1, 1);
    pids[1].setOutputLimits(-max_step, max_step);
    pids[2].setOutputLimits(-max_step, max_step);
    break;

  case Y:
    pids[0].setOutputLimits(-max_step, max_step);
    pids[1].setOutputLimits(-1, 1);
    pids[2].setOutputLimits(-max_step, max_step);
    break;

  case Z:
    pids[0].setOutputLimits(-max_step, max_step);
    pids[1].setOutputLimits(-max_step, max_step);
    pids[2].setOutputLimits(-1, 1);
    break;

  default:
    printf("Unknown PID direction of travel");
  }
}

// Compute PID values in force space in the direction of travel and for other
// PIDs whos measured values is outside the deadband range.
// Parameters:
//   direction: Direction of positional travel between the current path
//              point and next path point. Direction is a struct which has
//              both a value (X=0, Y=1, Z=2) and sign (+, -). Direction is
//              with respect to the tool frame.
//   pids: Vector containing the X, Y, Z PID controllers.
//   deadbands: X, Y, Z upper and lower force space deadbands (lower, upper).
//   pid_step_computed: Computed PID process values X, Y, Z for the
//                      current iteration.
void DeadbandCompute(const Direction& direction,
  vector<PID>& pids, const vector<pair<double, double>>& deadbands,
  vector<double>& pid_step_computed) {
  switch (direction.axis) {
  case X:
    pid_step_computed[0] = pids[0].compute();   
    // deadband filter Fy
    pid_step_computed[1] = pids[1].compute();
    if (InRange(pids[1].getProcessValue(), deadbands[1])) {
      pid_step_computed[1] = 0;
    }
    // deadband filter Fz
    pid_step_computed[2] = pids[2].compute();
    if (InRange(pids[2].getProcessValue(), deadbands[2])) {
      pid_step_computed[2] = 0;
    }
    break;
  case Y:
    // deadband filter Fx
    pid_step_computed[0] = pids[0].compute();
    if (InRange(pids[0].getProcessValue(), deadbands[0])) {
      pid_step_computed[0] = 0;
    }
    pid_step_computed[1] = pids[1].compute();
    
    // deadband filter Fz
    pid_step_computed[2] = pids[2].compute();
    if (InRange(pids[2].getProcessValue(), deadbands[2])) {
      pid_step_computed[2] = 0;
    }
    break;
  case Z:
    // deadband filter Fx
    pid_step_computed[0] = pids[0].compute();
    if (InRange(pids[0].getProcessValue(), deadbands[0])) {
      pid_step_computed[0] = 0;
    }

    // deadband filter Fy
    pid_step_computed[1] = pids[1].compute();
    if (InRange(pids[1].getProcessValue(), deadbands[1])) {
      pid_step_computed[1] = 0;
    }

    pid_step_computed[2] = pids[2].compute();
    break;
  default:
    printf("Incorrect direction selected");
    break;
  }
}


// E.g. map 0->1 to 0->255. 0.5 -> 127.5
double arduino_map(double value, double istart, double istop, double ostart, double ostop) {
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}



int ProcessPIDOutput(const Direction& direction,
  vector<double>& pid_step_computed, double slow, double fast) {
  switch (direction.axis) {
  case X:
    switch (direction.sign) {
    case 1: // (- Tool) -> (+ Forces)
      pid_step_computed[1] *= -1;
      pid_step_computed[2] *= -1;
      if (pid_step_computed[0] > 0) {
        pid_step_computed[0] = arduino_map(pid_step_computed[0], 0, 1, -slow - 0.0002, -fast - 0.0002);
      }
      else {
        pid_step_computed[0] = -slow - 0.0002;
        return 1; // set-point satisfied
      }
      break;
    case 0: // (+ Tool) -> (- Forces)
      pid_step_computed[1] *= -1;
      pid_step_computed[2] *= -1;
      if (pid_step_computed[0] < 0) {
        pid_step_computed[0] = arduino_map(pid_step_computed[0], 0, -1, slow + 0.0002, fast + 0.0002);
      }
      else {
        pid_step_computed[0] = slow + 0.0002;
        return 1; // set-point satisfied
      }
      break;
    }
    break;
  case Y:
    // process sign depending on direction (0-1 between max and min range)
    switch (direction.sign) {
    case 1: // (- Tool) -> (+ Forces)
      pid_step_computed[0] *= -1;
      pid_step_computed[2] *= -1;
      if (pid_step_computed[1] > 0) {
        pid_step_computed[1] = arduino_map(pid_step_computed[1], 0, 1, -slow, -fast);
      }
      else {
        pid_step_computed[1] = -slow;
        return 1; // set-point satisfied
      }
      break;
    case 0: // (+ Tool) -> (- Forces)
      pid_step_computed[0] *= -1;
      pid_step_computed[2] *= -1;
      if (pid_step_computed[1] < 0) {
        pid_step_computed[1] = arduino_map(pid_step_computed[1], 0, -1, slow, fast);
      }
      else {
        pid_step_computed[1] = slow;
        return 1; // set-point satisfied
      }
      break;
    }
    break;
  case Z:
    switch (direction.sign) {
    case 1: // (- Tool) -> (+ Forces)
      pid_step_computed[0] *= -1;
      pid_step_computed[1] *= -1;
      if (pid_step_computed[2] > 0) {
        pid_step_computed[2] = arduino_map(pid_step_computed[2], 0, 1, -slow + 0.0001, -fast + 0.0001);
	  }
      else {
        pid_step_computed[2] = -slow + 0.0001;
		  pid_step_computed[2] = -slow;
        return 1; // set-point satisfied
      }
      break;
    case 0: // (+ Tool) -> (- Forces)
      pid_step_computed[0] *= -1;
      pid_step_computed[1] *= -1;
      if (pid_step_computed[2] < 0) {
        pid_step_computed[2] = arduino_map(pid_step_computed[2], 0, -1, slow - 0.0001, fast - 0.0001);
      }
      else {
        pid_step_computed[2] = slow - 0.0001;
        return 1; // set-point satisfied
      }
      break;
    }
    break;
  }
  return 0;
}

void LogControlParameters(ofstream& os, milliseconds &ms, unsigned int& path_point,
  Direction& movement_direction, vector<PID>& pids,
  vector<pair<double, double>>& deadband, vector<vector<double>>& loads,
  vector<vector<double>>& positions) {

  // timestamp
  os << ms.count() << ",";

  // log path point
  os << path_point << ",";

  // log direction
  os << movement_direction.sign << "," << movement_direction.axis << ",";

  // P controller log gains values
  os << pids[0].getPParam() << "," << pids[1].getPParam() << "," <<
    pids[2].getPParam() << ",";

  // log deadbands
  os << deadband[0].first << "," << deadband[0].second << "," <<
    deadband[1].first << "," << deadband[1].second << "," <<
    deadband[2].first << "," << deadband[2].second << ",";

  os <<
    pids[0].getSetPoint() << "," <<
    pids[1].getSetPoint() << "," <<
    pids[2].getSetPoint() << ",";

  // log dsr position
  os <<
    positions[path_point + 1][0] << "," <<
    positions[path_point + 1][1] << "," <<
    positions[path_point + 1][2] << ",";

  // log desired loads
  os <<
    loads[path_point + 1][0] << "," <<
    loads[path_point + 1][1] << "," <<
    loads[path_point + 1][2] << "," <<
    loads[path_point + 1][3] << "," <<
    loads[path_point + 1][4] << "," <<
    loads[path_point + 1][5] << ",";
}
///////////////////////////////////////////////////////////////////////////////
////                               NAV BY                                  ////
///////////////////////////////////////////////////////////////////////////////

// Main algorithm as described in thesis. Move to a pose which satasfies
// a bending loads point. Once a point is found, record pose.
// Contains the following main subsystems:
//   - Bounding box
//   - Timeout between points
//   - Output file static location
//   - Output statistics file
//   - Delay between points option
//   - Deadbands
//   - 3x parrell PID's (X,Y,Z) 
//
// Parameters:
//   lwr: KUKA-LWR.
//   load_cell: Nano25E load cell.
//   follow_loads_path: Input file containing loads and positions.
//   output_filename: Output file.
//   point_delay: Delay between each path point.
int NavByBendingForces(LWR& lwr, Nano25E& load_cell,
  const string follow_loads_path, const string output_filename,
  const unsigned int point_delay) {
  // maximum number of steps between each path point iteration
  const unsigned int TIMEOUT_COUNT = 10000;
  // timestep for the PID controllers in seconds.
  const double PID_TIMESTEP = 0.002;
  // maximum PID controller output per iteration. Note: 0.001 is the LWR max in meters.
  const double PID_MAX_TRANSLATION = 0.00065; // (0.65mm)
  // minimum step size for direction of travel
  const double PID_MIN_PATH_TRANSLATION = 0.00055; // (0.55mm)
  // maximum load cell force value
  const double PID_MAX_FORCE = 25.0;
  // X, Y, Z safety boundary in m.
  const vector<double> SAFETY_BOUNDARY = { 1.012, 1.012, 1.007 };

  const string LOG_PATH_BASE =
    "C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA-LWR\\experiments\\";
  const string LOG_PATH_FILENAME = LOG_PATH_BASE +
    output_filename + Config::Extension::CSV;
  const string STATISTICS_FILENAME = LOG_PATH_BASE +
    output_filename + "_results" + Config::Extension::CSV;
  const string HEADER = "Timestamp,Point,Sign,Axis,PID-Px,PID-Py,PID-Pz,"
	  "Deadbands lower (0) and upper (1) Dx [low],Dx [high],Dy [low],Dy[high],Dz [low],Dz [high],SetPoint Fx, Fy, Fz,"
	  "Dsr Position Px,Py,Pz,Dsr Load Fx,Fy,Fz,Mx,My,Mz,Msr Load Fx,Fy,Fz,Mx,My,Mz,"
	  "External Load Fx,Fy,Fz,Mx,My,Mz,Msr Cartesian Pose R00,R01,R02,R03,R10,R11,R12,R13,R20,R21,R22,R23,"
	  "PID Pre P x,P y,P z,PID Post P x,P y,P z,Next Position Px,Py,Pz,\n";

  // initialize X, Y, Z parallel PID controllers with default control terms
  const vector<vector<vector<double>>> PID_TUNINGS = {
    //     PID_x           PID_y          PID_z
    { { 50, 0, 0 }, { 20, 0, 0 }, { 20, 0, 0 } },  // Fx travel
    { { 20, 0, 0 }, { 50, 0, 0}, { 20, 0, 0 } },  // Fy travel
    { { 20, 5, 0 }, { 20, 0, 0 }, { 50, 0, 0 } } };  // Fz travel

  // X travel: repeated path to tune PID_x
  // Y travel: repeated path to tune PID_y
  // Z travel: repeated path to tune PID_z

  vector<PID> pids = {
    PID(0, 0, 0, PID_TIMESTEP),
    PID(0, 0, 0, PID_TIMESTEP),
    PID(0, 0, 0, PID_TIMESTEP) };

  // working variables
  unsigned int err_val = 0;
  unsigned int path_point = 0;
  unsigned int timeout_counter = 0;
  Direction movement_direction;
  bool flag_path_direction_slowed = false;
  float load_cell_loads[6] = { 0, 0, 0, 0, 0, 0 };
  vector<vector<float>> found_points_pose;
  static vector<double> pid_step_computed = { 0, 0, 0 };
  vector<pair<double, double>> deadband = {
    pair<double, double>(), pair<double, double>(), pair<double, double>() };

  // open log file and write header
  ofstream os(LOG_PATH_FILENAME);
  os << HEADER;

  // set PID input limits to max acceptable force
  pids[0].setInputLimits(-PID_MAX_FORCE, PID_MAX_FORCE);
  pids[1].setInputLimits(-PID_MAX_FORCE, PID_MAX_FORCE);
  pids[2].setInputLimits(-PID_MAX_FORCE, PID_MAX_FORCE);

  // set PID output limits to max translations for step
  pids[0].setOutputLimits(-PID_MAX_TRANSLATION, PID_MAX_TRANSLATION);
  pids[1].setOutputLimits(-PID_MAX_TRANSLATION, PID_MAX_TRANSLATION);
  pids[2].setOutputLimits(-PID_MAX_TRANSLATION, PID_MAX_TRANSLATION);

  // get loads in path
  vector<vector<double>> loads;
  vector<vector<double>> positions;
  vector<Direction> tool_path;
  vector<vector<pair<double, double>>> deadbands;
  if (Reader::GetLoadsFromCSV(follow_loads_path, loads)) {
    return 1; // invalid input
  }
  if (Reader::GetPositionsFromCSV(follow_loads_path, positions)) {
    return 1; // invalid input
  }

  // use the position from the log file rather than config starting position
  // might have been minor offset from in the log file.
  Eigen::Matrix4d starting;
  starting <<
    Config::start_frame(0, 0), Config::start_frame(0, 1), Config::start_frame(0, 2), positions[0][0],
    Config::start_frame(1, 0), Config::start_frame(1, 1), Config::start_frame(1, 2), positions[0][1],
    Config::start_frame(2, 0), Config::start_frame(2, 1), Config::start_frame(2, 2), positions[0][2],
    0, 0, 0, 1;

  if (Reader::GetToolDirectionPathFromCSV(follow_loads_path, tool_path, starting)) {
    return 1; // invalid input
  }
  // calculate deadbands
  CalculateDeadbands(loads, deadbands);

  // calibrate, transform, start load cell
  if (load_cell.Initialize(
    Config::Filepath::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION,
    Config::SAMPLE_RATE,
    Config::LOADCELL_CHANNEL.c_str())) {
    return 2; // load cell initialization error
  }
  delay(1000);

  // start robot in cartesian impedance control mode
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  lwr.fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
  err_val = lwr.StartCartesianImpedanceControlMode(
    lwr.CARTESIAN_STIFFNESS_HIGH,
    lwr.CARTESIAN_DAMPING_LOW,
    est_ft);
  if (err_val != SUCCESS) {
    printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
    load_cell.Stop();
    return 3; // cartesian impedance control mode error
  }

  lwr.MoveToCartesianPose(starting);

  // set PID set points to first point in path
  pids[0].setSetPoint(loads[1][0]);
  pids[1].setSetPoint(loads[1][1]);
  pids[2].setSetPoint(loads[1][2]);

  // get starting process value
  load_cell.GetLoads(load_cell_loads);
  pids[0].setProcessValue(load_cell_loads[0]);
  pids[1].setProcessValue(load_cell_loads[1]);
  pids[2].setProcessValue(load_cell_loads[2]);

  // timestamp
  milliseconds ms;

  DBGPRINT("Starting Nav By Bending Forces");
  while (path_point < loads.size() - 1) {

    DBGPRINT("Path Point: %d", path_point);

    // set direction of travel
    movement_direction = tool_path[path_point];
    DBGPRINT("Direction: %d, sign %d", movement_direction.axis, movement_direction.sign);

    // set pid tunings depending on direction
    SetDirectionPIDTunings(PID_TUNINGS, movement_direction, pids);

    // set non-stop direction pid output values
    SetDirectionPIDOutputs(
      PID_MAX_TRANSLATION, PID_MIN_PATH_TRANSLATION, movement_direction, pids);

    // set deadbands
    deadband[0] = deadbands[path_point][0];
    deadband[1] = deadbands[path_point][1];
    deadband[2] = deadbands[path_point][2];

    // resolve current path point
    while (1) {
      lwr.fri_->WaitForKRCTick();

      // FRI connection check
      if (!lwr.fri_->IsMachineOK()) {
        printf("ERROR, the machine is not ready anymore\n");
        for (auto pose : found_points_pose) {
          DBGPRINT("%.5f, %.5f, %.5f\n", pose[3], pose[7], pose[11]);
        }
        load_cell.Stop();
        os.close();
        return 4;  // error machine not okay
      }

      // set FRI force term to self
      lwr.fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
      lwr.fri_->SetCommandedCartForcesAndTorques(est_ft);

      // get process loads, pose, and timestamp of acquisition
      load_cell.GetLoads(load_cell_loads);
      lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
      ms = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch());

      // set process value
      pids[0].setProcessValue(load_cell_loads[0]);
      pids[1].setProcessValue(load_cell_loads[1]);
      pids[2].setProcessValue(load_cell_loads[2]);

      LogControlParameters(os, ms, path_point, movement_direction, pids,
        deadband, loads, positions);

      // log measured loads
      LogLoad(os, load_cell_loads, PrettyPrint::ENDLINE::ENDCOMMA);

      // log external lwr loads
      LogLoad(os, est_ft, PrettyPrint::ENDLINE::ENDCOMMA);

      // log current full pose
      PrettyPrint::LogPose(os, lwr.pose_msr, PrettyPrint::ENDLINE::ENDCOMMA);

      // timeout if in sink
      if (timeout_counter++ >= TIMEOUT_COUNT) {
        load_cell.Stop();
        os.close();
        printf("Sink!\n");
        return Errors::ERROR_TIMEOUT_COUNT;  // error sink
      }

      // max bounds timeout check
      if (!InBounds(positions[path_point], lwr.pose_msr, SAFETY_BOUNDARY)) {
        load_cell.Stop();
        printf("Timeout"
        "Dsr: (%.5f, %.5f, %.5f) vs. Msr: (%.5f, %.5f, %.5f)\n",
          positions[path_point][0], positions[path_point][1],
          positions[path_point][2], lwr.pose_msr[3], lwr.pose_msr[7],
          lwr.pose_msr[11]);
        os <<
          0 << "," << 0 << "," << 0 << "," <<
          0 << "," << 0 << "," << 0 << "," <<
          0 << "," << 0 << "," << 0 << "\n";
        return Errors::ERROR_OUT_OF_BOUNDS; // error out of bounds
      }

      // compute PID error correction value using deadband
      // processes PID output for signs
      DeadbandCompute(movement_direction, pids, deadband, pid_step_computed);

      // log pre-adjusted PID correction
      os <<
        pid_step_computed[0] << "," <<
        pid_step_computed[1] << "," <<
        pid_step_computed[2] << ",";

      // ensure direction of travel always moving forwards
      if (ProcessPIDOutput(movement_direction, pid_step_computed, PID_MIN_PATH_TRANSLATION, PID_MAX_TRANSLATION)) {
        break;
      }

      // pid post processing
      os <<
        pid_step_computed[0] << "," <<
        pid_step_computed[1] << "," <<
        pid_step_computed[2] << ",";

      // convert PID force from tool frame to global coordinate system
      ForceMovement(lwr.pose_msr,
        { pid_step_computed[0],  // e.g. 0.5mm
          pid_step_computed[1],  // e.g. 0.1mm
          pid_step_computed[2],  // e.g. 0.0mm
          0, 0, 0 },
        lwr.pose_cmd);
      
      lwr.pose_cmd[0] = starting(0, 0);
      lwr.pose_cmd[1] = starting(0, 1);
      lwr.pose_cmd[2] = starting(0, 2);
      lwr.pose_cmd[4] = starting(1, 0);
      lwr.pose_cmd[5] = starting(1, 1);
      lwr.pose_cmd[6] = starting(1, 2);
      lwr.pose_cmd[8] = starting(2, 0);
      lwr.pose_cmd[9] = starting(2, 1);
      lwr.pose_cmd[10] = starting(2, 2);     

      // set next pose
      lwr.fri_->SetCommandedCartPose(lwr.pose_cmd);

      // log next position
      LogPosition(os, lwr.pose_cmd, PrettyPrint::ENDLINE::NEWLINE);
    }

    // found point

    // log PID
    os <<
      pid_step_computed[0] << "," <<
      pid_step_computed[1] << "," <<
      pid_step_computed[2] << ",";

    // log position
    LogPosition(os, lwr.pose_cmd, PrettyPrint::ENDLINE::NEWLINE);

    printf("Path Point %d. @: %.2f, %.2f, %.2f\n", path_point,
      lwr.pose_msr[3] * 1000, lwr.pose_msr[7] * 1000, lwr.pose_msr[11] * 1000);
    path_point++;
    printf("Loads: %.5f, %.5f, %.5f\n",
      load_cell_loads[0], load_cell_loads[1], load_cell_loads[2]);

    // set new set point for next points load values
    if (path_point != loads.size() - 1) {
      pids[0].setSetPoint(loads[path_point + 1][0]);
      pids[1].setSetPoint(loads[path_point + 1][1]);
      pids[2].setSetPoint(loads[path_point + 1][2]);
    }

    // reset timeout counter
    timeout_counter = 0;

    // log found point pose
    vector<float> v;
    for (unsigned int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
      v.push_back(lwr.pose_msr[i]);
    }
    found_points_pose.push_back(v);

    // delay between found points
    delay(point_delay);
  }

  // completion. Stop load cell and print found point poses
  load_cell.Stop();
  for (auto pose : found_points_pose) {
    DBGPRINT("%.5f, %.5f, %.5f\n", pose[3] - positions[0][0],
      pose[7] - positions[0][1], pose[11] - positions[0][2]);
  }

  return SUCCESS;
}
