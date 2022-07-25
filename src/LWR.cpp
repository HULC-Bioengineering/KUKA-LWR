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

// LWR.cpp interacts with the Fast Research Interface (FRI) along
// with move to Cartesian and joint commands. Also contains impedance
// contorl modes.
#include "LWR.h"
#include <iostream>
using std::unordered_map;
using std::vector;
using std::string;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;

using namespace std;

// joint space
const float LWR::JOINT_STIFFNESS_LOW[NUMBER_OF_JOINTS] = {
  0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f };
const float LWR::JOINT_STIFFNESS_HIGH[NUMBER_OF_JOINTS] = {
  10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f };
const float LWR::JOINT_DAMPING_LOW[NUMBER_OF_JOINTS] = {
  0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f };
const float LWR::JOINT_DAMPING_HIGH[NUMBER_OF_JOINTS] = {
  1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
const float LWR::JOINT_TORQUE_NONE[NUMBER_OF_JOINTS] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// cartesian space
const float LWR::CARTESIAN_STIFFNESS_LOW[NUMBER_OF_CART_DOFS] = {
  0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f };
const float LWR::CARTESIAN_STIFFNESS_HIGH[NUMBER_OF_CART_DOFS] = {
  10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f };
const float LWR::CARTESIAN_DAMPING_LOW[NUMBER_OF_CART_DOFS] = {
  0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f };
const float LWR::CARTESIAN_DAMPING_MID[NUMBER_OF_CART_DOFS] = {
  0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f };
const float LWR::CARTESIAN_DAMPING_HIGH[NUMBER_OF_CART_DOFS] = {
  1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
const float LWR::CARTESIAN_TORQUE_NONE[NUMBER_OF_CART_DOFS] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// DH parameters
const double LWR::D1 = 0.310;
const double LWR::D3 = 0.400;
const double LWR::D5 = 0.390;
const double LWR::D7 = 0.078;
const double LWR::ALPHA = PI / 2.0;
const double LWR::JNT_LIMITS[] = {
  170.0*PI / 180.0,  // 2.96705972839 RAD
  120.0*PI / 180.0,  // 2.09439510239 RAD
  170.0*PI / 180.0,  // 2.96705972839 RAD
  120.0*PI / 180.0,  // 2.09439510239 RAD
  170.0*PI / 180.0,  // 2.96705972839 RAD
  120.0*PI / 180.0,  // 2.09439510239 RAD
  170.0*PI / 180.0   // 2.96705972839 RAD
};

LWR::LWR() {
  // cycle time declared in init file
  fri_ = new FastResearchInterface(FRI_INIT.c_str());
}

LWR::LWR(char* init_file) {
  // cycle time declared in init file
  fri_ = new FastResearchInterface(init_file);
}

LWR::~LWR() {
  delete fri_;
}

void LWR::StopFRI() {
  fri_->StopRobot();
}

const char* LWR::GetCompleteRobotStateAndInformation() const {
  return fri_->GetCompleteRobotStateAndInformation();
}

void LWR::GetCurrentCartesianPose(float* pose) const {
  fri_->GetMeasuredCartPose(pose);
}

void LWR::GetCurrentJointPositions(float* joint_positions) const {
  fri_->GetMeasuredJointPositions(joint_positions);
}

void LWR::SetCartesianParameters(const float* stiffness,
  const float* damping, const float* torque) {
  if (stiffness != NULL) {
    fri_->SetCommandedCartStiffness(stiffness);
  }
  if (damping != NULL) {
    fri_->SetCommandedCartDamping(damping);
  }
  if (torque != NULL) {
    fri_->SetCommandedCartForcesAndTorques(torque);
  }
  fri_->WaitForKRCTick();
}

bool LWR::fkSolver(const std::vector<double>& jntPosDsr,
  double(&dsr_pose)[NUMBER_OF_FRAME_ELEMENTS],
  Eigen::Matrix4d* tool_frame) {
  // Local copy of the desired joint positions vector
  std::vector<double> jntPos(jntPosDsr);
  bool output = true;

  //Check requested angles are within the limits
  for (int i = 0; i < (int)jntPos.size(); i++){
    if (abs(jntPos[i]) > JNT_LIMITS[i]){
      jntPos[i] > 0 ? jntPos[i] = JNT_LIMITS[i] : jntPos[i] = -JNT_LIMITS[i];
      cout << "Warning! IK gives values out of bounds for joint " << i << endl;
      output = false;
    }
  }

  //Adjust to robot to FK coordinates from coordinates in the joint space
  jntPos[1] > 0 ? jntPos[1] -= PI : jntPos[1] += PI;
  jntPos[3] > 0 ? jntPos[3] -= PI : jntPos[3] += PI;
  jntPos[5] > 0 ? jntPos[5] -= PI : jntPos[5] += PI;
  jntPos[6] > 0 ? jntPos[6] -= PI : jntPos[6] += PI;

  Matrix4d F0tool;
  Matrix4d F01; DH(0.0, ALPHA, D1, jntPos[0], F01);
  Matrix4d F12; DH(0.0, ALPHA, 0.0, jntPos[1], F12);
  Matrix4d F23; DH(0.0, ALPHA, D3, jntPos[2], F23);
  Matrix4d F34; DH(0.0, ALPHA, 0.0, jntPos[3], F34);
  Matrix4d F45; DH(0.0, ALPHA, D5, jntPos[4], F45);
  Matrix4d F56; DH(0.0, ALPHA, 0.0, jntPos[5], F56);
  Matrix4d F67; DH(0.0, 0.0, D7, jntPos[6], F67);

  // Forward Kinematics transformation
  if (tool_frame != NULL) {
    F0tool = F01*F12*F23*F34*F45*F56*F67*(*tool_frame);
  }
  else {
    F0tool = F01*F12*F23*F34*F45*F56*F67;
  }

  // Generate pose
  dsr_pose[0] = F0tool(0, 0);
  dsr_pose[1] = F0tool(0, 1);
  dsr_pose[2] = F0tool(0, 2);
  dsr_pose[3] = F0tool(0, 3);
  dsr_pose[4] = F0tool(1, 0);
  dsr_pose[5] = F0tool(1, 1);
  dsr_pose[6] = F0tool(1, 2);
  dsr_pose[7] = F0tool(1, 3);
  dsr_pose[8] = F0tool(2, 0);
  dsr_pose[9] = F0tool(2, 1);
  dsr_pose[10] = F0tool(2, 2);
  dsr_pose[11] = F0tool(2, 3);

  return output;
}

// not yet functional
bool LWR::ikSolver(const double(&dsr_pose)[NUMBER_OF_FRAME_ELEMENTS],
  std::vector<double>& jntPosDsr) {

  bool output = true;  // return value
  jntPosDsr = std::vector<double>(7, 0.0);  // initialize

  // Auxiliar variables
  double mod_pW, mod_pWxy, c2, s2, c3, s3;

  // Getting the transformation matrix
  Matrix3d R;
  R << dsr_pose[0], dsr_pose[1], dsr_pose[2],
    dsr_pose[4], dsr_pose[5], dsr_pose[6],
    dsr_pose[8], dsr_pose[9], dsr_pose[10];
  Vector3d p;
  p << dsr_pose[3], dsr_pose[7], dsr_pose[11];

  Vector3d Runitz;
  Runitz << dsr_pose[2], dsr_pose[6], dsr_pose[10];
  cout << "Runitz" << Runitz << "\n";

  // Inverse kinematics calculations. Closed form solution.
  // Based in the solution for an anthropomorfic arm with an spherical wrist.
  // Extracted from Siciliano et al (2010)

  // wrist position
  Vector3d pW;
  pW = p - D7 * Runitz;
  cout << "pW " << pW << "\n";

  // Calculate wrist position
  jntPosDsr[0] = std::atan2(pW[1], pW[0]);
  printf("atan2 jnt0: %.7f", jntPosDsr[0]);
  cout << "Norm pW: %.7f" << pW.norm() << "\n";
  mod_pW = pow(pW.norm(), 2);
  

  c3 = (mod_pW - D3*D3 - D5*D5) / (2 * D3*D5);
  printf("c3: %.5f\n", c3);
  // If c3>1, there is no solution for IKT
  if (c3>1){
    output = false;
    cout << "Attempt to access to a point out of the workspace." << endl;
    jntPosDsr = std::vector<double>(7, 0.0);
    return output;
  }

  s3 = -sqrt(1 - c3*c3);
  jntPosDsr[3] = atan2(s3, c3) + PI / 2;

  // We do not use the extra dof for getting the inverse kinematics
  jntPosDsr[2] = 0.0;

  mod_pWxy = sqrt(pW[0] * pW[0] + pW[1] * pW[1]);
  s2 = ((D3 + D5*c3)*pW[2] - D5*s3*mod_pWxy) / mod_pW;
  c2 = ((D3 + D5*c3)*mod_pWxy + D5*s3*pW[2]) / mod_pW;
  jntPosDsr[1] = atan2(s2, c2);

  // Calculate orientation (angles of the wrist joints)
  Matrix3d T01;
  Matrix3d T12;
  Matrix3d T23;
  Matrix3d pose03;
  Matrix3d pose36;

  T01 << cos(jntPosDsr[0]), 0.0, sin(jntPosDsr[0]),
    sin(jntPosDsr[0]), 0.0, -cos(jntPosDsr[0]),
    0.0, 1.0, 0.0;
  T12 << cos(jntPosDsr[1]), -sin(jntPosDsr[1]), 0.0,
    sin(jntPosDsr[1]), cos(jntPosDsr[1]), 0.0,
    0.0, 0.0, 1.0;
  T23 << cos(jntPosDsr[3]), 0.0, sin(jntPosDsr[3]),
    sin(jntPosDsr[3]), 0.0, -cos(jntPosDsr[3]),
    0.0, 1.0, 0.0;

  pose03 = T01*T12*T23;
  pose36 = pose03.inverse() * R;

  jntPosDsr[4] = atan2(pose36(1, 2), pose36(0, 2));
  jntPosDsr[5] = atan2(sqrt(pose36(0, 2)*pose36(0, 2) + 
    pose36(1, 2)*pose36(1, 2)), pose36(2, 2));
  jntPosDsr[6] = atan2(pose36(2, 1), -pose36(2, 0));

  //Adjust to robot from IK coordinates
  // (keeping joint coord. within the interval [-pi,pi])
  jntPosDsr[1] < -PI / 2 ? jntPosDsr[1] += 3 * PI / 2 : jntPosDsr[1] -= PI / 2;
  jntPosDsr[3] < -PI / 2 ? jntPosDsr[3] += 3 * PI / 2 : jntPosDsr[3] -= PI / 2;
  jntPosDsr[6] <     0 ? jntPosDsr[6] += PI : jntPosDsr[6] -= PI;

  jntPosDsr[3] = -jntPosDsr[3]; //Correcting for the RobotRotation

  for (int i = 0; i < (int)jntPosDsr.size(); i++){
    if (abs(jntPosDsr[i]) > JNT_LIMITS[i]){
      output = false;
      printf("%d: %.7f\n", i, jntPosDsr[i]);
      jntPosDsr[i] > 0 ? jntPosDsr[i] = JNT_LIMITS[i] :
        jntPosDsr[i] = -JNT_LIMITS[i];
      cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
    }
  }
  return output;
}

// http://www.letworyinteractive.com/blendercode/d1/d16/frames_8hpp_source.html#l00299
// http://www.letworyinteractive.com/blendercode/d1/d4c/frames_8cpp_source.html
void LWR::DH(double a, double alpha, double d, double theta, Matrix4d& FXX) {
  double ct, st, ca, sa;
  ct = cos(theta);
  st = sin(theta);
  sa = sin(alpha);
  ca = cos(alpha);

  FXX <<
    ct, -st*ca, st*sa, a*ct,
    st, ct*ca, -ct*sa, a*st,
    0, sa, ca, d,
    0, 0, 0, 1;
}

///////////////////////////////////////////////////////////////////////////////
////                       STARTING CONTROL MODES                          ////
///////////////////////////////////////////////////////////////////////////////
int LWR::StartJointImpedanceControlMode(
  const float(&desired_stiffness)[NUMBER_OF_JOINTS],
  const float(&desired_damping)[NUMBER_OF_JOINTS],
  const float(&desired_torque)[NUMBER_OF_JOINTS]) {

  int err_val = EOK;
  if ((fri_->GetCurrentControlScheme() !=
    FastResearchInterface::JOINT_IMPEDANCE_CONTROL) || (!fri_->IsMachineOK())){
    DBGPRINT("Switching mode to joint impedance control");
    fri_->StopRobot();
    fri_->GetMeasuredJointPositions(joint_msr);
    fri_->SetCommandedJointPositions(joint_msr);
    err_val = fri_->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
  }
  fri_->SetCommandedJointStiffness(desired_stiffness);
  fri_->SetCommandedJointDamping(desired_damping);
  fri_->SetCommandedJointTorques(desired_torque);
  fri_->WaitForKRCTick();
  return err_val;
}

int LWR::StartCartesianImpedanceControlMode(
  const float(&desired_stiffness)[NUMBER_OF_CART_DOFS],
  const float(&desired_damping)[NUMBER_OF_CART_DOFS],
  const float(&desired_torque)[NUMBER_OF_CART_DOFS]) {
  
  int err_val = EOK;
  if ((fri_->GetCurrentControlScheme() !=
    FastResearchInterface::CART_IMPEDANCE_CONTROL) || (!fri_->IsMachineOK())) {
    DBGPRINT("Switching mode to cartesian impedance control");
    if ((fri_->GetCurrentControlScheme() !=
      FastResearchInterface::JOINT_IMPEDANCE_CONTROL) ||
      (!fri_->IsMachineOK())) {
      fri_->StopRobot();
      fri_->SetCommandedJointStiffness(JOINT_STIFFNESS_HIGH);
      fri_->SetCommandedJointDamping(JOINT_DAMPING_HIGH);
      fri_->GetMeasuredJointPositions(joint_msr);
      fri_->SetCommandedJointPositions(joint_msr);
      fri_->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
      fri_->SetCommandedJointStiffness(JOINT_STIFFNESS_HIGH);
      fri_->SetCommandedJointDamping(JOINT_DAMPING_HIGH);
      fri_->WaitForKRCTick();
    }
    fri_->StopRobot();
    fri_->SetCommandedCartStiffness(desired_stiffness);
    fri_->SetCommandedCartDamping(desired_damping);
    fri_->SetCommandedCartForcesAndTorques(desired_torque);
    err_val = fri_->StartRobot(FastResearchInterface::CART_IMPEDANCE_CONTROL);
  }
  fri_->SetCommandedCartStiffness(desired_stiffness);
  fri_->SetCommandedCartDamping(desired_damping);
  fri_->SetCommandedCartForcesAndTorques(desired_torque);
  fri_->WaitForKRCTick();
  return err_val;
}

int LWR::StartJointPositionControlMode(
  const float(&desired_stiffness)[NUMBER_OF_JOINTS],
  const float(&desired_damping)[NUMBER_OF_JOINTS],
  const float(&desired_torque)[NUMBER_OF_JOINTS]) {

  int err_val = EOK;
  if ((fri_->GetCurrentControlScheme() !=
    FastResearchInterface::JOINT_POSITION_CONTROL) || (!fri_->IsMachineOK())) {
    DBGPRINT("Switching mode to joint position control");
    fri_->StopRobot();
    fri_->GetMeasuredJointPositions(joint_msr);
    fri_->SetCommandedJointPositions(joint_msr);
    err_val = fri_->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
  }
  fri_->SetCommandedJointStiffness(desired_stiffness);
  fri_->SetCommandedJointDamping(desired_damping);
  fri_->SetCommandedJointTorques(desired_torque);
  fri_->WaitForKRCTick();
  return err_val;
}

///////////////////////////////////////////////////////////////////////////////
////                            MOVE HELPERS                               ////
///////////////////////////////////////////////////////////////////////////////
bool InThreshold(const float(&err_tolerance)[3],
  const float (&lax_threshold)[3], float pid_x, float pid_y, float pid_z) { 
  int count = 0;
  int lax_count = 0;
  abs(pid_x) < err_tolerance[0] ? count++ : abs(pid_x) < lax_threshold[0] ?
    lax_count++ : lax_count;
  abs(pid_y) < err_tolerance[1] ? count++ : abs(pid_y) < lax_threshold[1] ?
    lax_count++ : lax_count;
  abs(pid_z) < err_tolerance[2] ? count++ : abs(pid_z) < lax_threshold[2] ?
    lax_count++ : lax_count;
  return (count == 2 && lax_count == 1) || (count == 3);
}

static void GetQuaternionStep(const Eigen::Quaternionf& qa,
  const Eigen::Quaternionf& qb, const float max_angle,
  Eigen::Quaternionf& qres) {
  float distance = qa.angularDistance(qb);
  if (distance > max_angle) {
    qres = qa.slerp(0.01f, qb);
    qres.normalize();
    float step_distance = qa.angularDistance(qres);
    float max_bounded_slerp_percent = max_angle / step_distance;
    qres = qa.slerp(max_bounded_slerp_percent, qb);
  }
  else {
    qres = qb;
    qres.normalize();
  }
}

int LWR::MoveToCartesianPose(Eigen::Matrix4d& frame, Eigen::Matrix3d* rotation_matrix,
  const unsigned int point_delay) {
    vector<float> pose = {
      static_cast<float>(frame(0, 0)),
      static_cast<float>(frame(0, 1)),
      static_cast<float>(frame(0, 2)),
      static_cast<float>(frame(0, 3)),
      static_cast<float>(frame(1, 0)),
      static_cast<float>(frame(1, 1)),
      static_cast<float>(frame(1, 2)),
      static_cast<float>(frame(1, 3)),
      static_cast<float>(frame(2, 0)),
      static_cast<float>(frame(2, 1)),
      static_cast<float>(frame(2, 2)),
      static_cast<float>(frame(2, 3)),
    };
    return MoveToCartesianPose(pose, rotation_matrix, point_delay);
}

int LWR::MoveToCartesianPose(vector<float> pose,
  Eigen::Matrix3d* rotation_matrix,
  const unsigned int point_delay) {
  float arr_position[NUMBER_OF_FRAME_ELEMENTS];
  std::copy(pose.begin(), pose.end(), arr_position);
  return MoveToCartesianPose(arr_position, rotation_matrix, 0);
}

int LWR::MoveToCartesianPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
  Eigen::Matrix3d* rotation_matrix, const unsigned int point_delay) {
    const float POS_TOLERANCE[3] = { 0.00003f, 0.00003f, 0.00003f};
    const float MAX_ITERATION_STEP = 0.0009f;
    const float ROT_TOLERANCE = 0.001f; // quaternion angular displacement
    const float ROT_MAX_ITERATION_STEP = 0.008f; // max quaternion step angular
    int err_val = SUCCESS;
    const float SLACK_VAR[3] = { 0.00002f, 0.00002f, 0.00002f };
    Eigen::Matrix3f cur;
    Eigen::Quaternionf qres;

    float torques[7] = {0,0,0,0,0,0,0};

    // translation control by PID
    PID pid_x = PID(1200, 2, 0, 0.002);
    pid_x.setOutputLimits(-MAX_ITERATION_STEP - 0.0002, MAX_ITERATION_STEP + 0.0002);
    PID pid_y = PID(1200, 2, 0, 0.002);
    pid_y.setOutputLimits(-MAX_ITERATION_STEP, MAX_ITERATION_STEP);
    PID pid_z = PID(1200, 2, 0, 0.002);
    pid_z.setOutputLimits(-MAX_ITERATION_STEP, MAX_ITERATION_STEP);

    // start robot in cartesian impedance control mode
    float est_ft[6] = { 0, 0, 0, 0, 0, 0 };
    fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
    err_val = StartCartesianImpedanceControlMode(
      CARTESIAN_STIFFNESS_HIGH, CARTESIAN_DAMPING_LOW, est_ft);
    if (err_val != SUCCESS) {
      printf("ERROR, could not start in Cartesian Impedance Control Mode\n");
      return err_val;
    }

    // get measured pose
    fri_->GetMeasuredCartPose(pose_msr);

    // set starting orientation
    Eigen::Matrix3f start;
    if (rotation_matrix != NULL) {
      start = (*rotation_matrix);
    }
    else {
      start <<
        pose_msr[0], pose_msr[1], pose_msr[2],
        pose_msr[4], pose_msr[5], pose_msr[6],
        pose_msr[8], pose_msr[9], pose_msr[10];
    }
    Eigen::Quaternionf qa(start);
    // set desired orientation
    Eigen::Matrix3f end;
    end << pose[0], pose[1], pose[2],
      pose[4], pose[5], pose[6],
      pose[8], pose[9], pose[10];
    Eigen::Quaternionf qb(end);

    // set position PID process
    pid_x.setProcessValue(pose_msr[3]);
    pid_y.setProcessValue(pose_msr[7]);
    pid_z.setProcessValue(pose_msr[11]);

    // set position PID setpoint
    pid_x.setSetPoint(pose[3]);
    pid_y.setSetPoint(pose[7]);
    pid_z.setSetPoint(pose[11]);

    DBGPRINT("POSITION CONTROL");
    while (!InThreshold(POS_TOLERANCE, SLACK_VAR, abs(pose[3] - pose_msr[3]),
      abs(pose[7] - pose_msr[7]), abs(pose[11] - pose_msr[11])))
    {
      // FRI connection check
      if (!fri_->IsMachineOK()) {
        printf("ERROR, the machine is not ready anymore\n");
        return ERROR_MACHINE_NOT_OKAY;
      }

      //fri_->GetMeasuredJointTorques(torques);
      //f << "p" << torques[0] << "," << torques[1] << "," << torques[2] << "," << torques[3] << "," << torques[4] << "," << torques[5] << "," << torques[6] << "\n";

      // set FRI force term to self
      fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
      fri_->SetCommandedCartForcesAndTorques(est_ft);

      // set next position process values
      fri_->GetMeasuredCartPose(pose_msr);
      pid_x.setProcessValue(pose_msr[3]);
      pid_y.setProcessValue(pose_msr[7]);
      pid_z.setProcessValue(pose_msr[11]);

      // compute and add to measured pose
      pose_msr[3] += static_cast<float>(pid_x.compute());
      pose_msr[7] += static_cast<float>(pid_y.compute());
      pose_msr[11] += static_cast<float>(pid_z.compute());

      pose_msr[0] = pose[0];
      pose_msr[1] = pose[1];
      pose_msr[2] = pose[2];

      pose_msr[4] = pose[4];
      pose_msr[5] = pose[5];
      pose_msr[6] = pose[6];

      pose_msr[8] = pose[8];
      pose_msr[9] = pose[9];
      pose_msr[10] = pose[10];

      // send pose to KRC
      fri_->SetCommandedCartPose(pose_msr);
      fri_->WaitForKRCTick();

      // set new process values
      fri_->GetMeasuredCartPose(pose_msr);
      pid_x.setProcessValue(pose_msr[3]);
      pid_y.setProcessValue(pose_msr[7]);
      pid_z.setProcessValue(pose_msr[11]);
    }

    const float position_x = pose_msr[3];
    const float position_y = pose_msr[7];
    const float position_z = pose_msr[11];
    
    /*
    DBGPRINT("ORIENTATION CONTROL");
    while (qa.angularDistance(qb) > ROT_TOLERANCE) {
      if (!fri_->IsMachineOK()) {
        printf("ERROR, the machine is not ready anymore\n");
        return ERROR_MACHINE_NOT_OKAY;
      }

      //fri_->GetMeasuredJointTorques(torques);
      //f << "a" << torques[0] << "," << torques[1] << "," << torques[2] << "," << torques[3] << "," << torques[4] << "," << torques[5] << "," << torques[6] << "\n";

      // set FRI force term to self
      fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
      fri_->SetCommandedCartForcesAndTorques(est_ft);

      // set next position process values
      fri_->GetMeasuredCartPose(pose_msr);
      start <<
        pose_msr[0], pose_msr[1], pose_msr[2],
        pose_msr[4], pose_msr[5], pose_msr[6],
        pose_msr[8], pose_msr[9], pose_msr[10];
      qa = start;
      GetQuaternionStep(qa, qb, ROT_MAX_ITERATION_STEP, qres);
      cur = qres.toRotationMatrix();
      pose_msr[0] = cur(0, 0);
      pose_msr[1] = cur(0, 1);
      pose_msr[2] = cur(0, 2);
      pose_msr[3] = position_x;
      pose_msr[4] = cur(1, 0);
      pose_msr[5] = cur(1, 1);
      pose_msr[6] = cur(1, 2);
      pose_msr[7] = position_y;
      pose_msr[8] = cur(2, 0);
      pose_msr[9] = cur(2, 1);
      pose_msr[10] = cur(2, 2);
      pose_msr[11] = position_z;

      // send pose to KRC
      fri_->SetCommandedCartPose(pose_msr);
      fri_->WaitForKRCTick();

      // set start quaternion to new orientation
      start <<
        pose_msr[0], pose_msr[1], pose_msr[2],
        pose_msr[4], pose_msr[5], pose_msr[6],
        pose_msr[8], pose_msr[9], pose_msr[10];
      qa = start;
    }
    */
    // ms
    for (unsigned int i = 0; i < point_delay; i+= 2) {
      // set FRI force term to self
      fri_->SetCommandedCartPose(pose_msr);
      fri_->GetEstimatedExternalCartForcesAndTorques(est_ft);
      fri_->SetCommandedCartForcesAndTorques(est_ft);
      fri_->WaitForKRCTick();
    }

    // diff
    DBGPRINT("Abs Diff: %.3f, %.3f, %.3f",
      abs(abs(pose[3]) - abs(pose_msr[3])) * 1000,
      abs(abs(pose[7] - abs(pose_msr[7]))) * 1000,
      abs(abs(pose_msr[11]) - abs(pose[11])) * 1000);

    DBGPRINT("Moved to Position: %.3f, %.3f, %.3f",
      pose_msr[3]*1000, pose_msr[7]*1000, pose_msr[11]*1000);
    //f << pose_msr[3] * 1000 << "," << pose_msr[7] * 1000 << "," << pose_msr[11] * 1000 << "\n";
    //f.close();
    return SUCCESS;
  }

int LWR::MoveToJointPosition(const vector<float>& position, AngleUnit a) {
  float arr_position[NUMBER_OF_JOINTS];
  std::copy(position.begin(), position.end(), arr_position);
  return MoveToJointPosition(arr_position, a);
}

int LWR::MoveToJointPosition(const float(&position)[NUMBER_OF_JOINTS],
  AngleUnit a) {
  int err_val = EOK;

  // IRML handles the acceleration, velociy, and position
  TypeIRML RML = TypeIRML(NUMBER_OF_JOINTS, fri_->GetFRICycleTime());
  TypeIRMLInputParameters IP = TypeIRMLInputParameters(NUMBER_OF_JOINTS);
  TypeIRMLOutputParameters OP = TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

  // Start in joint position control mode
  if ((fri_->GetCurrentControlScheme() !=
    FastResearchInterface::JOINT_POSITION_CONTROL) || (!fri_->IsMachineOK())) {
    DBGPRINT("Switching mode to position control");
    fri_->StopRobot();
    fri_->GetMeasuredJointPositions(joint_msr);
    fri_->SetCommandedJointPositions(joint_msr);
    err_val = fri_->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
  }
  if ((err_val != EOK) && (err_val != EALREADY)) {
    printf("An error occurred during starting up the robot...\n");
    return 1;
  }

  fri_->SetCommandedJointStiffness(JOINT_STIFFNESS_HIGH);
  fri_->SetCommandedJointDamping(JOINT_DAMPING_LOW);
  fri_->WaitForKRCTick();

  fri_->GetMeasuredJointPositions(joint_msr);

  // Populate input parameters with current robot state information
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    IP.CurrentPosition->VecData[i] = (
      static_cast<double>(DEG(joint_msr[i])));

    if (a == RAD) {
      IP.TargetPosition->VecData[i] = static_cast<double>(DEG(position[i]));
    }
    else {
      IP.TargetPosition->VecData[i] = static_cast<double>(position[i]);
    }

    IP.MaxVelocity->VecData[i] = static_cast<double>(50.0);
    IP.MaxAcceleration->VecData[i] = static_cast<double>(50.0);
    IP.SelectionVector->VecData[i] = true;
  }
  // While moving to position
  err_val = TypeIRML::RML_WORKING;
  while ((fri_->IsMachineOK()) &&
    (err_val != TypeIRML::RML_FINAL_STATE_REACHED)) {
    fri_->WaitForKRCTick();
    // Get next motion state information using IP and save into OP
    err_val = RML.GetNextMotionState_Position(IP, &OP);
    if ((err_val != TypeIRML::RML_WORKING) &&
      (err_val != TypeIRML::RML_FINAL_STATE_REACHED)) {
      printf("ERROR, during trajectory generation (%d).", err_val);
      return -1;
    }

    // Set next step information
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
      joint_msr[i] = static_cast<float>(RAD(static_cast<double>(
        OP.NewPosition->VecData[i])));
    }
    fri_->SetCommandedJointPositions(joint_msr);

    // Set new input parameters to output parameters
    *(IP.CurrentPosition) = *(OP.NewPosition);
    *(IP.CurrentVelocity) = *(OP.NewVelocity);
  }
  return EOK;
}
