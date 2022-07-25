#include "Scripts.h"
#include <Eigen/StdVector>

using Config::start_frame;

int FollowPath(LWR& lwr, Nano25E& load_cell, string follow_path_filename, string output_loads_filename,
  ControlMode mode) {
  unsigned int err_val = SUCCESS;
  const unsigned int POINT_DELAY = 2000;  // ms
  float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
  float load_cell_loads[6] = { 0, 0, 0, 0, 0, 0 };
  int count = 0;
  vector<float> tmp = {};
  vector<vector<float>> loads;
  vector<vector<double>> pos_rot;
  vector<float> load_val;

  const std::string input = \
    "-i \"C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\paths\\position\\first_cut_square.ngc\"";
  const std::string options = "-z 1 -x 1 -y 1";
  const std::string output = "-o \"C:\\Users\\HMMS\\Documents\\GitHub\\Thesis\\KUKA LWR\\paths\\position\\first_cut_square.csv\"";

  MotionParser mp = MotionParser();
  mp.SetInputFile(follow_path_filename);
  ofstream of(output_loads_filename);

  // generating tool path
  vector<Eigen::Vector3d> path;
  std::string cmd = Config::Filepath::python_path + " " + Config::Filepath::parser_path + " " + input + " " + output + " " + options;
  //std::string input_tool_path = Utils::exec(cmd.c_str());

  std::ifstream t(follow_path_filename);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string input_tool_path = buffer.str();
  
  Parser::ParseToolPath(input_tool_path, path);
  for (auto point : path) {
    printf("%.5f %.5f %.5f\n", point[0], point[1], point[2]);
  }


  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> frame_path;
  frame_path.reserve(path.size());
  for (Eigen::Vector3d point : path) {
    Eigen::Matrix4d path_point;
    path_point <<
      start_frame(0, 0), start_frame(0, 1), start_frame(0, 2), start_frame(0, 3) + point[0],
      start_frame(1, 0), start_frame(1, 1), start_frame(1, 2), start_frame(1, 3) + point[1],
      start_frame(2, 0), start_frame(2, 1), start_frame(2, 2), start_frame(2, 3) + point[2],
      0, 0, 0, 1;
    frame_path.push_back(path_point);
  }
  Eigen::Matrix3d rotation_mat;
  rotation_mat <<
    start_frame(0, 0), start_frame(0, 1), start_frame(0, 2),
    start_frame(1, 0), start_frame(1, 1), start_frame(1, 2),
    start_frame(2, 0), start_frame(2, 1), start_frame(2, 2);
  
  vector<vector<float>> joint_path;

  // calibrate, transform, start load cell
  err_val = load_cell.Initialize(Config::Filepath::LOAD_CELL_CALIBRATION_PATH.c_str(),
    Config::LOAD_CELL_TRANSFORMATION,
    Config::SAMPLE_RATE,
    Config::LOADCELL_CHANNEL.c_str());
  if (err_val != 0) {
    printf("Failed to initialize load cell\n");
    return err_val;
  }
  delay(2000);

  switch (mode) {
  case CARTESIAN:

    // start robot in cartesian impedance control mode   
    lwr.fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    err_val = lwr.StartCartesianImpedanceControlMode(
      lwr.CARTESIAN_STIFFNESS_HIGH, lwr.CARTESIAN_DAMPING_LOW, est_ft);
    if (err_val != SUCCESS) {
      printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
      load_cell.Stop();
      return err_val;
    }

    DBGPRINT("Starting Path Follow");
    count = 0;
    for (Eigen::Matrix4d cart_pose : frame_path) {
      printf("Point %d / %d\n", count, frame_path.size() - 1);
      printf("Moving to: %.3f, %.3f, %.3f\n",
        cart_pose(0,3)*1000, cart_pose(1,3)*1000, cart_pose(2,3)*1000);
      
      lwr.MoveToCartesianPose(cart_pose, &rotation_mat, POINT_DELAY);
      lwr.MoveToCartesianPose(cart_pose, &rotation_mat, POINT_DELAY);

      load_cell.GetLoads(load_cell_loads);
      tmp.assign(load_cell_loads, load_cell_loads + 6);
      loads.push_back(tmp);

      DBGPRINT("Forces: %.5f, %.5f, %.5f",
        load_cell_loads[0], load_cell_loads[1], load_cell_loads[2]);
      lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
      pos_rot.push_back({
        lwr.pose_msr[3], lwr.pose_msr[7], lwr.pose_msr[11],
      });
      count++;
    }
    // move back to starting position
    lwr.MoveToCartesianPose(frame_path[0]);
    break;

  case JOINT:
    // parse log file for joint path
    err_val = mp.ParseMotion(&joint_path, NULL);
    if (err_val != SUCCESS) {
      printf("ERROR, could not parse input file\n");
      return err_val;
    }
    loads.reserve(joint_path.size());
    pos_rot.reserve(joint_path.size());

    // setup joint position control mode with mid level stiffness and damping
    err_val = lwr.StartJointPositionControlMode(lwr.JOINT_STIFFNESS_HIGH,
      lwr.JOINT_DAMPING_HIGH, lwr.JOINT_TORQUE_NONE);
    if (err_val != SUCCESS) {
      printf("ERROR, could not start in Joint Position Control Mode\n");
      return err_val;
    }

    DBGPRINT("Starting Path Follow");
    count = 0;
    for (vector<float> joint_positions : joint_path) {
      printf("Point %d / %d\n", count, joint_path.size() - 1);
      lwr.MoveToJointPosition(joint_positions);
      delay(POINT_DELAY);

      load_cell.GetLoads(load_cell_loads);
      tmp.assign(load_cell_loads, load_cell_loads + 6);
      loads.push_back(load_val);

      lwr.fri_->GetMeasuredCartPose(lwr.pose_msr);
      //GetXYZEulerRotationsFromPose(lwr.pose_msr, rot);
      pos_rot.push_back({
        lwr.pose_msr[3], lwr.pose_msr[7], lwr.pose_msr[11],
        //rot[0], rot[1], rot[2]
      });
      count++;
    }
    // move back to starting position
    lwr.MoveToJointPosition(joint_path[0]);
    break;

  }
  delay(POINT_DELAY);
  Logging::WriteLoadsAndPositions(of, loads, pos_rot);

  load_cell.Stop();
  return EOK;
}