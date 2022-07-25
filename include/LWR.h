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

// LWR.h interacts with the Fast Research Interface (FRI) and handles all
// connection, communication, and movement of the KUKA robot.
#ifndef LWR_H_
#define LWR_H_

#include "PID.h"
#include <FastResearchInterface.h>
#include <OSAbstraction.h>
#include <TypeIRML.h>

#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>

#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>
#include <queue>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifndef PI
#define PI  3.14159265358979323846264338327950288419
#endif

#ifndef DEG
#define DEG(A) ((A)*180.0/PI)
#endif

#ifndef RAD
#define RAD(A) ((A)*PI/180.0)
#endif

#define DEBUG true

#define FILENAME (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#define DBGPRINT(M, ...) \
  if (DEBUG) fprintf(stderr, "DBG %s(%d) " M "\n", FILENAME, __LINE__, ##__VA_ARGS__)

class LWR {
private:
  // default calibration file location
  const std::string FRI_INIT = \
    "../../calibration/980039-FRI-Driver.init";

  /// DH parameters
  static const double D1;
  static const double D3;
  static const double D5;
  static const double D7;
  static const double ALPHA;

  // rotation limits of each joint (rad)
  static const double JNT_LIMITS[];

  // dofs and frame elements
  static const unsigned int NUMBER_OF_FRAME_ELEMENTS = 12;
  static const unsigned int NUMBER_OF_JOINTS = 7;
  static const unsigned int NUMBER_OF_CART_DOFS = 6;

  // return values
  static const unsigned int ERROR_MACHINE_NOT_OKAY = 1;
  static const unsigned int SUCCESS = 0;

  // Forward kinematic solver.
  // Parameters:
  //   joints_dsr: desired joint positions.
  //   frame: resulting frame from fk of joint positions.
  //   tool_frame: additional frame describing current tool.
  static bool fkSolver(const std::vector<double>& joints_dsr,
    double(&frame)[NUMBER_OF_FRAME_ELEMENTS],
    Eigen::Matrix4d* tool_frame = NULL);

  // Inverse kinematic solver.
  // Warning: Current incomplete/inoperable. #TODO (matthewbstokes) fix.
  // Parameters:
  //   
  static bool ikSolver(const double(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    std::vector<double>& jntPosDsr);

  // Construction of a DH matrix from the four underlying components.
  // Forward kinematics helper.
  // Parameters:
  //   a:
  //   alpha:
  //   d:
  //   theta: 
  //   FXX: resulting 4x4 transformation matrix.
  static void DH(double a, double alpha, double d, double theta,
    Eigen::Matrix4d& FXX);

public:
  // angle input unit
  enum AngleUnit { DEG, RAD };
  
  // #TODO (matthewbstokes) should not be able to interact with this directly.
  FastResearchInterface* fri_;

  // buffered storage
  float joint_msr[NUMBER_OF_JOINTS];
  float joint_cmd[NUMBER_OF_JOINTS];
  float pose_msr[NUMBER_OF_FRAME_ELEMENTS];
  float pose_cmd[NUMBER_OF_FRAME_ELEMENTS];

  // preset joint and cartesian dampening, stiffness, and torques
  const static float JOINT_DAMPING_LOW[NUMBER_OF_JOINTS];
  const static float JOINT_DAMPING_HIGH[NUMBER_OF_JOINTS];
  const static float JOINT_STIFFNESS_LOW[NUMBER_OF_JOINTS];
  const static float JOINT_STIFFNESS_HIGH[NUMBER_OF_JOINTS];
  const static float JOINT_TORQUE_NONE[NUMBER_OF_JOINTS];
  const static float CARTESIAN_DAMPING_LOW[NUMBER_OF_CART_DOFS];
  const static float CARTESIAN_DAMPING_MID[NUMBER_OF_CART_DOFS];
  const static float CARTESIAN_DAMPING_HIGH[NUMBER_OF_CART_DOFS];
  const static float CARTESIAN_STIFFNESS_LOW[NUMBER_OF_CART_DOFS];
  const static float CARTESIAN_STIFFNESS_HIGH[NUMBER_OF_CART_DOFS];
  const static float CARTESIAN_TORQUE_NONE[NUMBER_OF_CART_DOFS];

  // constructor
  LWR();
  LWR(char* init_file);

  // destructor
  ~LWR();

   // FRISTART in joint position control mode.
   // Parameters:
   //   stiffness: N/m
   //   damping: Nm/s
   //   torque: 
   int StartJointPositionControlMode(
     const float(&stiffness)[NUMBER_OF_JOINTS],
     const float(&damping)[NUMBER_OF_JOINTS],
     const float(&torque)[NUMBER_OF_JOINTS]);

   // FRISTART in joint impedance control mode.
   // Parameters:
   //   stiffness: N/m
   //   damping: Nm/s
   //   torque: 
   int StartJointImpedanceControlMode(
     const float(&stiffness)[NUMBER_OF_JOINTS],
     const float(&damping)[NUMBER_OF_JOINTS],
     const float(&torque)[NUMBER_OF_JOINTS]);

   // FRISTART in cartesian impedance control mode.
   // Parameters:
   //   stiffness: N/m
   //   damping: Nm/s
   //   torque: 
   int StartCartesianImpedanceControlMode(
     const float(&stiffness)[NUMBER_OF_CART_DOFS],
     const float(&damping)[NUMBER_OF_CART_DOFS],
     const float(&torque)[NUMBER_OF_CART_DOFS]);

   // Update Cartesian control mode parameters.
   // Parameters:
   //   stiffness: N/m
   //   damping: Nm/s
   //   torques:
   void LWR::SetCartesianParameters(const float* stiffness = NULL,
     const float* damping = NULL, const float* torques = NULL);

   // Get current Cartesian pose.
   // Parameters:
   //   pose: fills the first 12 elements with the cartesian pose.
   void GetCurrentCartesianPose(float* pose) const;

   // Get current Cartesian pose.
   // Parameters:
   //   pose: fills the first 7 elements with the joint positions.
   void GetCurrentJointPositions(float* joint_positions) const;

   // Get all the robot information.
   // Returns:
   //   massive string will all the information you could ask for.
   const char* GetCompleteRobotStateAndInformation() const;

  // Move the LWR to the input joint positions.
  // Parameters:
  //   position: vector of joint positions. {A1, A2, A3, A4, A5, A6, E1}.
  //   a: unit of input joint position (DEG or RAD).
  int MoveToJointPosition(const std::vector<float>& position,
    AngleUnit a=RAD);

  // Move the LWR to the input joint positions.
  // Parameters:
  //   position: array of joint positions. {A1, A2, A3, A4, A5, A6, E1}.
  //   a: unit of input joint position (DEG or RAD).
  int MoveToJointPosition(const float(&position)[NUMBER_OF_JOINTS],
    AngleUnit a=RAD);

  // Move the LWR to the input cartesian frame.
  // Highest level abstraction of the MoveToCartesianPose function.
  // Uses an internal PID controller.
  // Parameters:
  //   frame: desired cartesian frame.
  //   point_delay: pause once reaching point to reduce dynamic effects.
  int MoveToCartesianPose(Eigen::Matrix4d& frame,
    Eigen::Matrix3d* rotation_matrix = NULL, const unsigned int point_delay = 0);

  // Move the LWR to the input cartesian pose. Uses an internal PID controller.
  // #TODO (matthewbstokes) remove rotation_matrix its un-needed.
  // Parameters:
  //   pose: row major transformation matrix as array
  //   point_delay: pause once reaching point to reduce dynamic effects.
  int MoveToCartesianPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS],
    Eigen::Matrix3d* rotation_matrix = NULL, const unsigned int point_delay=0);

  // Move the LWR to the input cartesian pose. Uses an internal PID controller.
  // #TODO (matthewbstokes) remove rotation_matrix its un-needed.
  // Parameters:
  //   pose: row major transformation matrix as vector
  //   point_delay: pause once reaching point to reduce dynamic effects.
  int MoveToCartesianPose(std::vector<float> pose,
    Eigen::Matrix3d* rotation_matrix = NULL, const unsigned int point_delay=0);

  // Terminates FRI connection
  void StopFRI();
};
#endif  // C:_USERS_HMMS_FRI_FRILIBRARY_WINDOWS_KUKA_LWR_H_

