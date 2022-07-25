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
#include "LWRTest.h"
#include "utils.h"

#include <utility>
#include <boost/numeric/ublas/matrix.hpp>
using boost::numeric::ublas::matrix;
using std::vector;
using std::pair;
using std::make_pair;
using std::string;

using BoostMatrix::BuildMatrixFromVector;
using BoostMatrix::BuildPoseMatrixFromArray;
using PrettyPrint::PrintTestCase;
using Utils::PoseToString;
using Utils::Round;


void LWRTest::Init() {
    BuildMatrixFromVector(4, 4, &default_matrix, &current_pose_matrix);
  }

void LWRTest::ResetDefaultConditions() {
    // joint_position
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
      joint_position[i] = 0;
    }
    // cart_pose
    for (int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; ++i) {
      cart_pose[i] = 0;
    }
    // transformation, load cell, scale
    for (int i = 0; i < NUMBER_OF_CART_DOFS; ++i) {
      transformation[i] = 0;
      loads[i] = 0;
      scale[i] = 1;
    }
    // matrix and lwr
    Init();
  }

LWRTest::LWRTest(LWR* lwr): joint_position() {
  lwr_ = lwr;
  default_matrix = vector<float>{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  current_pose_matrix = matrix<float>(4, 4);
  ResetDefaultConditions();
}
/*
bool LWRTest::TestIsEqualPose(const float(&solution)[NUMBER_OF_FRAME_ELEMENTS], const float(&cart_pose)[NUMBER_OF_FRAME_ELEMENTS]) {
  for (int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++) {
    if (Round(solution[i],8) != Round(cart_pose[i],8)) {
      printf("// FAILED\n");
      printf("Input:\n%sSolution:\n%s\n", PoseToString(cart_pose).c_str(), PoseToString(solution).c_str());
      return false;
    }
  }
  printf("// PASSED\n");
  return true;
}

void LWRTest::TestLoadMovementHelper(string test_desc, Load load, float value, float (&current_pose)[12]) {
    PrintTestCase("TestingLoadMovement", test_desc);
    ResetDefaultConditions();
    loads[load] = value;
    lwr_->ForceMovement(current_pose, scale, loads, cart_pose);
  }


void LWRTest::TestLoadMovementHelper(string test_desc, Load load, float value) {
  PrintTestCase("TestingLoadMovement", test_desc);
  ResetDefaultConditions();
  loads[load] = value;
  lwr_->ForceMovement(current_pose_matrix, scale, loads, cart_pose);
}

  // preserves order
void LWRTest::TestLoadMovementHelper(string test_desc, vector<pair<Load, float>>* set_loads) {
    PrintTestCase("TestingLoadMovement", test_desc);
    ResetDefaultConditions();
    for (std::vector<pair<Load, float>>::iterator it = set_loads->begin(); it != set_loads->end(); ++it) {
      loads[it->first] = it->second;
    }
    lwr_->ForceMovement(current_pose_matrix, scale, loads, cart_pose);
  }
  

void LWRTest::TestLoadMovementHelper(string test_desc, vector<pair<Load, float>>* set_loads, float(&current_pose)[12]) {
  PrintTestCase("TestingLoadMovement", test_desc);
  ResetDefaultConditions();
  for (std::vector<pair<Load, float>>::iterator it = set_loads->begin(); it != set_loads->end(); ++it) {
    loads[it->first] = it->second;
  }
  lwr_->ForceMovement(current_pose, scale, loads, cart_pose);
}


void LWRTest::TestLoadMovementXForce() {
  string test = "XForce\t 10N";
  float load_value = 10;
  Load load = FX;
  float sol[] = {1, 0, 0, load_value,
                 0, 1, 0, 0,
                 0, 0, 1, 0 };
  TestLoadMovementHelper(test, load, load_value);
  TestIsEqualPose(sol, cart_pose);
}

void LWRTest::TestLoadMovementYForce() {
  float load_value = 10;
  Load load = FY;
  float sol[] = {1, 0, 0, 0,
                 0, 1, 0, load_value,
                 0, 0, 1, 0 };
  TestLoadMovementHelper("YForce\t 10N", load, load_value);
  TestIsEqualPose(sol, cart_pose);
}

void LWRTest::TestLoadMovementZForce() {
  float load_value = 10;
  Load load = FZ;
  float sol[] = {1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, load_value };
  TestLoadMovementHelper("ZForce\t 10N", load, load_value);
  TestIsEqualPose(sol, cart_pose);
}


void LWRTest::TestLoadMovementXYZForce() {
  float load_value = 10;
  vector<pair<Load, float>> v = { make_pair(FX, load_value),
    make_pair(FY, load_value), make_pair(FZ, load_value) };
  float sol[] = {1, 0, 0, load_value,
                 0, 1, 0, load_value,
                 0, 0, 1, load_value };
  TestLoadMovementHelper("XYZForce\t 10N each", &v);
  TestIsEqualPose(sol, cart_pose);
}

void LWRTest::TestLoadMovementXYZForceWithPose() {
  float load_value = 10;
  vector<pair<Load, float>> v = { make_pair(FX, load_value),
    make_pair(FY, load_value), make_pair(FZ, load_value) };
  float pose[] = {
    -0.06406f, -0.99784f, 0.01427f, -0.41020f,
    0.12690f, -0.02233f, -0.99166f, -0.37526f,
    0.98984f, -0.06171f, 0.12806f, 0.31982f
  };
  float sol[] = { 1, 0, 0, load_value,
    0, 1, 0, load_value,
    0, 0, 1, load_value };
  TestLoadMovementHelper("XYZForce\t 10N each", &v, pose);
  TestIsEqualPose(sol, cart_pose);
}



void LWRTest::TestLoadMovementXMoment() {
  float load_value = 90;
  Load load = MX;
  float sol[] = { 1, 0, 0, 0,
                  0, 0, -1, 0,
                  0, 1, 0, 0 };
  TestLoadMovementHelper("XMoment\t 90deg", load, load_value);
  TestIsEqualPose(sol, cart_pose);
}

void LWRTest::TestLoadMovementYMoment() {
  float load_value = 90;
  Load load = MY;
  float sol[] = {0, 0, 1, 0,
                 0, 1, 0, 0,
                 -1, 0, 0, 0 };
  TestLoadMovementHelper("YMoment\t 90deg", load, load_value);
  TestIsEqualPose(sol, cart_pose);
}

void LWRTest::TestLoadMovementZMoment() {
  float load_value = 90;
  Load load = MZ;
  float sol[] = {0, -1, 0, 0,
                 1, 0, 0, 0,
                 0, 0, 1, 0 };
  TestLoadMovementHelper("ZMoment\t 90deg", load, load_value);
  TestIsEqualPose(sol, cart_pose);
}


void LWRTest::TestLoadMovementXYZMoment() {
  float load_value = 90;
  vector<pair<Load, float>> v = { make_pair(MX, load_value),
    make_pair(MY, load_value), make_pair(MZ, load_value) };
  float sol[] = {0, 0, 1, 0,
                 0, 1, 0, 0,
                 -1, 0, 0, 0 };
  TestLoadMovementHelper("XYZMoment\t 90deg each", &v);
  TestIsEqualPose(sol, cart_pose);
}


void LWRTest::TestLoadMovementZ45Moment() {
  float load_value = 90;
  Load load = MZ;
  float pose[] = {
      -0.06406f, -0.99784f, 0.01427f, -0.41020f,
      0.12690f, -0.02233f, -0.99166f, -0.37526f,
      0.98984f, -0.06171f, 0.12806f, 0.31982f
  };
  float sol[] = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0 };
  TestLoadMovementHelper("Pure z moment 45", load, load_value, pose);
  TestIsEqualPose(sol, cart_pose);
}


void LWRTest::RunAll() {
  TestLoadMovementXForce();
  TestLoadMovementYForce();
  TestLoadMovementZForce();
  //TestLoadMovementXYZForce();
  TestLoadMovementXMoment();
  TestLoadMovementYMoment();
  TestLoadMovementZMoment();
  //TestLoadMovementXYZMoment();
  //TestLoadMovementZ45Moment();
  //TestLoadMovementXYZForceWithPose();
}
*/