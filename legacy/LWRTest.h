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

// Testing for the class LWR.cpp
#include "LWR.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <vector>
#include <string>
#include <utility>

enum Load { FX, FY, FZ, MX, MY, MZ };
class LWRTest {
private:
  float joint_position[NUMBER_OF_JOINTS];
  float cart_pose[NUMBER_OF_FRAME_ELEMENTS];
  float transformation[NUMBER_OF_CART_DOFS];
  float loads[NUMBER_OF_CART_DOFS];
  float scale[NUMBER_OF_CART_DOFS];
  std::vector<float> default_matrix;
  boost::numeric::ublas::matrix<float> current_pose_matrix;
  boost::numeric::ublas::matrix<float> pose_solution;
  LWR *lwr_;

  void Init();
  void ResetDefaultConditions();
  bool TestIsEqualPose(const float(&solution)[NUMBER_OF_FRAME_ELEMENTS], const float(&cart_pose)[NUMBER_OF_FRAME_ELEMENTS]);

public:
  LWRTest(LWR* lwr);
  /*
  void TestLoadMovementHelper(std::string, Load load, float value);
  void TestLoadMovementHelper(std::string test_desc, Load load, float value, float(&current_pose)[NUMBER_OF_FRAME_ELEMENTS]);
  void TestLoadMovementHelper(std::string test_desc, std::vector<std::pair<Load, float>>* set_loads);
  void TestLoadMovementHelper(std::string test_desc, std::vector<std::pair<Load, float>>* set_loads, float(&current_pose)[NUMBER_OF_FRAME_ELEMENTS]);

  

   
  // LoadMovement Tests
  void TestLoadMovementXForce();
  void TestLoadMovementYForce();
  void TestLoadMovementZForce();
  void TestLoadMovementXYZForce();
  void TestLoadMovementXMoment();
  void TestLoadMovementYMoment();
  void TestLoadMovementZMoment();
  void TestLoadMovementXYZMoment();
  void TestLoadMovementZ45Moment();
  void TestLoadMovementXYZForceWithPose();

  void RunAll();
  */
};