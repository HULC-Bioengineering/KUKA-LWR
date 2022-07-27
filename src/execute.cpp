// Copyright 2016 Corey Smith. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//   http ://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: coreysmith03@hotmail.com (Corey Smith)

// execute.cpp is the main executable.
#include "Config.h"
#include "LWR.h"
#include "Utils.h"
#include "Statistics.h"
#include "Scripts.h"
#include <oaidl.h>
#include <atlsafe.h>
#include <iomanip>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>
#define _USE_MATH_DEFINES		// Get definition of M_PI (value of Pi)
#include <math.h>
#include <list>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <chrono>

#pragma warning ( disable : 4005 )

using namespace std;
using Errors::ERROR_INCORRECT_NUM_ELEMENTS;
using Errors::ERROR_INVALID_SELECTION;
using Parser::ListPointsInLoadThreshold;
using PrettyPrint::PrintPrompt;
using PrettyPrint::PrintJoints;
using PrettyPrint::PrintPose;
using PrettyPrint::PrintLoad;
using std::cin;
using std::cout;
using std::make_pair;
using std::map;
using std::ofstream;
using std::pair;
using std::string;
using std::to_string;
using std::transform;
using std::vector;
using Utils::SplitLine;
using namespace std::chrono;
using namespace Config::Filepath;
using namespace Config::Extension;
using namespace HexapodCOMLib;					// Namespace of the MHP

///////////////////////////////////////////////////////////////////////////////
////                           ENUMS & MAPPINGS                            ////
///////////////////////////////////////////////////////////////////////////////
enum Programs {
	RECORD,
	REPLAY,
	NAVBYBENDINGFORCES,
	FREEMOVEMENT,
	FREEMOVEMENTPOSELOADRECORD,
	FOLLOWPATH,
	MOVETOJOINTPOSITION,
	MOVETOCARTESIANPOSE,
	GETJOINTPOSITION,
	GETCARTESIANPOSE,
	GETJOINTANGLESANDCARTESIANPOSE,
	GETEXTERNALLOADCELLLOADS,
	GETROBOTINFORMATION,
	STARTPOSITIONCONTROLMODE,
	STARTCARTESIANIMPEDANCECONTROLMODE,
	INITIALIZELOADCELL,
	STOPLOADCELL,
	GENERATEPATHSTATS,
	DEBUGGING,
	STOPFRI,
	PLOT,
	MHPOPEN,
	MHPZERO,
	MHPSTOP,
	MHPPOSITIONCONTROL,
	MHPRUNPATH,
	MHPNAVBYBENDINGFORCES,
	MHPGETCURRENTPOS,
	MHPLOADCELLTRANSFORM,
	MHPMANUALLOADCELL,
	MHPMOVEBACK,
	MHPCNC,
	MHPRETRACTSAFE,
	TESTCODE,
    TERMINATE
  //add new scripts here
};

static map<Programs, string> programs = {
  { STARTPOSITIONCONTROLMODE, "Start Position Control Mode" },
  { STARTCARTESIANIMPEDANCECONTROLMODE, "Start Cart Impedance Control Mode" },
  { MOVETOJOINTPOSITION, " Move To Joint Position" },
  { MOVETOCARTESIANPOSE, " Move To Cartesian Pose" },
  { FREEMOVEMENT, " Free Movement" },
  { FREEMOVEMENTPOSELOADRECORD, " Free Movement Load and Pose Record" },
  { FOLLOWPATH, " Follow Path" },
  { RECORD, " Record" },
  { REPLAY, " Replay" },
  { NAVBYBENDINGFORCES, " Nav By Bending Forces" },
  { GETJOINTPOSITION, " Get Joint Position" },
  { GETCARTESIANPOSE, " Get Cartesian Pose" },
  { GETJOINTANGLESANDCARTESIANPOSE, "Get Path Point" },
  { GETROBOTINFORMATION, "Get Robot Information" },
  { INITIALIZELOADCELL, "Initialize Load Cell" },
  { GETEXTERNALLOADCELLLOADS, "Get External Load Cell Loads" },
  { STOPLOADCELL, "Stop Load Cell" },
  { GENERATEPATHSTATS, "Generate Path Stats" },
  { DEBUGGING, "Debugging" },
  { MHPOPEN, "MHP Open" },
  { MHPZERO, "MHP Zero" },
  { MHPSTOP, "MHP STOP!!!" },
  { MHPPOSITIONCONTROL, "MHP Choose Position" },
  { MHPRUNPATH, "MHP Run Path" },
  { MHPNAVBYBENDINGFORCES, "MHP Nav By Bending Forces" },
  { MHPLOADCELLTRANSFORM, "MHP Joystick" },
  { MHPMANUALLOADCELL, "MHP Manual Load Cell Movement" },
  { MHPGETCURRENTPOS, "MHP Get Current Pose" },
  { MHPMOVEBACK, "MHP Move Back" },
  { MHPCNC, "MHP CNC Cartesian" },
  { STOPFRI, "End FRI Connection" },
  { PLOT, "Generate Plot from Log File"},
  { MHPRETRACTSAFE, "MHP Retrack New Robot to Safe Position" },
  { TESTCODE, "Code For Testing Code" },
  { TERMINATE, "Terminate" }
  //add new scripts here with what you would like the user to see
};

enum CartesianPoseOptions {
  TRANSFORMATION,
  TRANSLATION,
  PRESET
};

static map<CartesianPoseOptions, string> pose_options = {
    { TRANSFORMATION, "Transformation Matrix (4x3)" },
    { TRANSLATION, "Translation X,Y,Z" },
    { PRESET, "Preset Cartesian Pose" }
};

///////////////////////////////////////////////////////////////////////////////
////                          PRESET POSITIONS                             ////
///////////////////////////////////////////////////////////////////////////////

// #TODO need to set values not currently set
static float WEDGE_JOINT[NUMBER_OF_JOINTS] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static float WEDGE_CARTESIAN[NUMBER_OF_FRAME_ELEMENTS] = {
  1.0f, 0.0f, 0.0f, 0.0000f,
  0.0f, 1.0f, 0.0f, 0.0000f,
  0.0f, 0.0f, 1.0f, 0.0000f
};

static map<string, pair<float*, float*>> PRESET_POSITIONS = {
  { "no_wedge",
  make_pair(WEDGE_CARTESIAN, WEDGE_JOINT) },
};

///////////////////////////////////////////////////////////////////////////////
////                      FUNCTION FORWARD DECLARIATIONS                   ////
///////////////////////////////////////////////////////////////////////////////
void Record(LWR& lwr);
void Replay(LWR& lwr);
void FollowPath(LWR& lwr, Nano25E& lc);
void GetJointPosition(LWR& lwr);
void GetCartesianPose(LWR& lwr);
void FreeCartesianMovement(LWR& lwr);
void FreeMovementPoseLoadRecord(LWR& lwr, Nano25E& lc);
void StartCartesianImpedanceControlMode(LWR& lwr);
void StartPositionControlMode(LWR& lwr);
void NavByBendingForces(LWR& lwr, Nano25E& lc);
void GetLoadCellLoads(Nano25E& lc);
void GetJointAnglesAndCartesianPose(LWR& lwr);
void MoveToJointPosition(LWR& lwr);
void MoveToCartesianPose(LWR& lwr, bool pid);
void GeneratePathStats();
void GeneratePlots();
void GeneratePlots(string, string);
void Debug(LWR& lwr, Nano25E& lc);
void Clear();
//add new scripts to this list
//====================== Mini Hexapod Platform Functions ======================//
void MHPOpen(IHexapodPtr& mhp, int robot);												//connects, opens anf homes MHP
void MHPZero(IHexapodPtr& mhp, int robot);												//zeros MHP
void MHPStop(IHexapodPtr& mhp, int robot);												//stops MHP
void MHPPositionControl(IHexapodPtr& mhp, int robot);									//allows absolute or relative motions
void MHPRunPath(IHexapodPtr& mhp, Nano25E& lc, int robot);								//runs a programmed path from a csv file
void MHPNavByBendingForces(IHexapodPtr& mhp, Nano25E& lc, int robot);					//runs a loads csv file
void MHPPIDLoadCell(IHexapodPtr& mhp, Nano25E& lc, int robot);							//proper transformations of loadcell
void MHPManualLoadCell(IHexapodPtr& mhp, Nano25E& lc, int robot);						//manual inputing of loads
void MHPMoveBack(IHexapodPtr& mhp, Nano25E& lc, int robot);								//move mhp back to give clearance
void TestCode(IHexapodPtr& mhp, Nano25E& lc, int robot);								//Function where test code can go
void MHPcnc(IHexapodPtr& mhp, Nano25E& lc, int robot);									//Function to run cartesian path without need of recording loads or delays
void MHPRetract(IHexapodPtr& mhp, Nano25E& lc, int robot);								//Move new robot to safe retracted position
//=========================== Conversion  Functions ===========================//
_variant_t VectorToVariant(vector<double>& vec);										//converts vector t-matrix to variant which can be used by MHP
vector<double> VariantToVector(const _variant_t& var);									//converts variant that comes from MHP to a T-matrix vector
vector<double> TFlipToTNorm(vector<double>& flip);										//converts T matrix flip (X,Y,Z in last row MHP standard) to T matrix normal (X,Y,Z in last column)
vector<double> TNormToTFlip(vector<double>& flip);										//converts T matrix normal (X,Y,Z in last column) to T matrix flip (X,Y,Z in last row MHP standard)
vector<double> DoubleVariantToVector(const _variant_t& var);							//Converts variant that is stored as a 2D matrix, ex. converting the get position function of MHP, to a 1d vector 16 elements
_variant_t RightToLeft(const _variant_t& var, float loads[6]);							//Converts from the right to left coordinate system, just rotations and needs the angles or loads that were used to convert
_variant_t LeftToRight(const _variant_t& var, float loads[6]);							//Converts from the left to right coordinate system, just rotations and needs the angles or loads that were used to convert
void MHPPrintCurrentPos(IHexapodPtr& mhp, int robot);												//prints the current positions of the MHP if the current position has already been gathered
vector<vector<double>> MHPSetLowerDeadbands();											//sets the lower deadbands for MHP
vector<vector<double>> MHPSetUpperDeadbands(IHexapodPtr& mhp);							//sets the upper deadbands for MHP
vector<vector<double>> MHPSetLimits();													//sets limits on how close the load difference needs to be before NavByBending moves onto next load point
vector<vector<double>> MHPSetLimits2();													//sets second broader limits on how close the load difference needs to be before NavByBending moves onto next load point
vector<double> MHPSetPControl();														//sets the P control for the MHP
void MHPNerdStats(vector<vector<vector<double>>>& complete_loads, int& length, int& loops, string& savestring);		//function to calculate mean and standard deviations of the loads measured during run path
vector<double> MHPTranspose(vector<double>& matrix);									//transposes a vector (matrix) because MHP uses the transpose for some reason
vector<double> MHPDigitize(IHexapodPtr& mhp, Nano25E& lc);								//used to digitize the scapula during Calibration and then transform coordinate system
void MHPVelocity(IHexapodPtr& mhp, Nano25E& lc);										//Ask and set velocity of mhp
void MHP_ID_Controller(vector<double> time, vector<vector<double>> error_all, vector<double>& integral, vector<double>& derivative);								//function for ID controller

///////////////////////////////////////////////////////////////////////////////
////                                 MAIN                                  ////
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  int selection = -1;
  LWR lwr = LWR();
  Nano25E lc = Nano25E();														//LoadCell variable
  IHexapodPtr mhp;																//MHP variable that communicates to the robot
  HRESULT hr = ::CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
  hr = mhp.CreateInstance(__uuidof(HexapodCOMLib::Hexapod));					//Needed to make the MHP object
  int robot = 0;																//old robot = 1; new robot = 2;

  MHPOpen(mhp, robot);

  while (true) {

    // program selection
    fprintf(stdout, "SELECT A PROGRAM\n");
    for (auto const &it : programs) {
      printf("(%d) %s\n", it.first, it.second.c_str());
    }
    PrintPrompt();
    cin >> selection;
    switch (selection) {
    case RECORD:
      Record(lwr);
      break;
    case REPLAY:
      Replay(lwr);
      break;
    case FOLLOWPATH:
      FollowPath(lwr, lc);
      break;
    case GETROBOTINFORMATION:
      printf("%s\n", lwr.GetCompleteRobotStateAndInformation());
      break;
    case GETJOINTPOSITION:
      GetJointPosition(lwr);
      break;
    case GETCARTESIANPOSE:
      GetCartesianPose(lwr);
      break;
    case GETJOINTANGLESANDCARTESIANPOSE:
      GetJointAnglesAndCartesianPose(lwr);
      break;
    case FREEMOVEMENT:
      FreeCartesianMovement(lwr);
      break;
    case FREEMOVEMENTPOSELOADRECORD:
      FreeMovementPoseLoadRecord(lwr, lc);
      break;
    case STARTCARTESIANIMPEDANCECONTROLMODE:
      StartCartesianImpedanceControlMode(lwr);
      break;
    case STARTPOSITIONCONTROLMODE:
      StartPositionControlMode(lwr);
      break;
    case NAVBYBENDINGFORCES:
      NavByBendingForces(lwr, lc);
      break;
    case INITIALIZELOADCELL:
      lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
        Config::LOAD_CELL_TRANSFORMATION,
        Config::SAMPLE_RATE,
        Config::LOADCELL_CHANNEL);
      break;
    case GETEXTERNALLOADCELLLOADS:
      GetLoadCellLoads(lc);
      break;
    case STOPLOADCELL:
      lc.Stop();
      break;
    case MOVETOJOINTPOSITION:
      MoveToJointPosition(lwr);
      break;
    case MOVETOCARTESIANPOSE:
      MoveToCartesianPose(lwr, false);
      break;
    case STOPFRI:
      lwr.StopFRI();
      break;
    case TERMINATE:
      lwr.StopFRI();
      return SUCCESS;
    case GENERATEPATHSTATS:
      GeneratePathStats();
      break;
    case PLOT:
      GeneratePlots();
      break;
    case DEBUGGING:
      Debug(lwr, lc);
      break;
      //add new scripts here to be added to list
	case MHPOPEN:
	  MHPOpen(mhp, robot);
      break;
	case MHPZERO:
	  MHPZero(mhp, robot);
	  break;
	case MHPSTOP:
	  MHPStop(mhp, robot);
	  break;
	case MHPPOSITIONCONTROL:
	  MHPPositionControl(mhp, robot);
	  break;
	case MHPRUNPATH:
	  MHPRunPath(mhp, lc, robot);
	  break;
	case MHPNAVBYBENDINGFORCES:
	  MHPNavByBendingForces(mhp, lc, robot);
	  break;
	case MHPGETCURRENTPOS:
	  MHPPrintCurrentPos(mhp, robot);
	  break;
	case MHPLOADCELLTRANSFORM:
	  MHPPIDLoadCell(mhp, lc, robot);
	  break;
	case MHPMANUALLOADCELL:
	  MHPManualLoadCell(mhp, lc, robot);
	  break;
	case MHPMOVEBACK:
	  MHPMoveBack(mhp, lc, robot);
	  break;
	case MHPCNC:
	  MHPcnc(mhp, lc, robot);
	  break;
	case MHPRETRACTSAFE:
	  MHPRetract(mhp, lc, robot);
	case TESTCODE:
	  TestCode(mhp, lc, robot);
	  break;
    default:
      fprintf(stdout, "Incorrect option selected\n");
      Clear();
      break;
    }
    selection = -1;
  }
  return SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
////                              FUNCTIONS                                ////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
////                       RECORD & REPLAY & FOLLOW                    ////
///////////////////////////////////////////////////////////////////////////
void Record(LWR& lwr) {
  string log_file_name;
  unsigned int stop_condition;
  unsigned int record_mode;

  fprintf(stdout, "Enter recorded file name\n> ");
  cin >> log_file_name;
  fprintf(stdout, "Select joint(%d) or cartesian(%d) control mode\n> ",
    JOINT, CARTESIAN);
  cin >> record_mode;
  fprintf(stdout, "Select stopping condition:\n\
                  AO Angle(%d)\n\
                  Time(%d)\n\
                  Key Press(%d)\n> ",
                  A0_ANGLE, TIME, KEY_PRESS);
  cin >> stop_condition;

  RecordMotion(lwr, log_file_name, static_cast<ControlMode>(record_mode),
    static_cast<RecordStopCondition>(stop_condition));
}

void Replay(LWR& lwr) {
  string log_file_name;
  unsigned int record_mode;

  fprintf(stdout, "Select file from %s to replay\n> ",
    LOGS_PATH.c_str());
  cin >> log_file_name;
  fprintf(stdout, "Select joint(%d) or cartesian(%d) control mode\n> ",
    JOINT, CARTESIAN);
  cin >> record_mode;

  ReplayMotion(lwr, LOGS_PATH + log_file_name + DAT,
    static_cast<ControlMode>(record_mode));
}

// #TODO (matthewbstokes) check for filename's existance and extension to know
// how we must parse.
void FollowPath(LWR& lwr, Nano25E& lc) {
  string output_loads_filename;
  string follow_path;
  string batch_filename;
  unsigned int number_of_trials;
  unsigned int points_per_trial = 1; // #TODO fix me

  fprintf(stdout, "What path would you like to follow?\n> ");
  cin >> follow_path;
  fprintf(stdout, "How many trials?\n> ");
  cin >> number_of_trials;
  fprintf(stdout, "Please enter output loads file name?\n> ");
  cin >> output_loads_filename;

  // execute trials
  for (unsigned int i = 0; i < number_of_trials; ++i) {

    batch_filename = EXP_PATH + output_loads_filename + to_string(i) + CSV;
    printf("%s\n", batch_filename.c_str());
    
    FollowPath(lwr, lc,
      PATHS_POSITION_PATH + follow_path + CSV, batch_filename, CARTESIAN);
  }
  // generate repeatability stats file
  Stats::GenerateNerdStatsFile(EXP_PATH + output_loads_filename, CSV,
    number_of_trials, points_per_trial);
}

///////////////////////////////////////////////////////////////////////////
////                          GETTERS & SETTERS                        ////
///////////////////////////////////////////////////////////////////////////
void GetJointPosition(LWR& lwr) {
  float joint_position[NUMBER_OF_JOINTS];
  lwr.GetCurrentJointPositions(joint_position);
  PrintJoints(joint_position);
}

void GetCartesianPose(LWR& lwr) {
  float cart_pose[NUMBER_OF_FRAME_ELEMENTS];
  lwr.GetCurrentCartesianPose(cart_pose);
  PrintPose(cart_pose);
}

void GetLoadCellLoads(Nano25E& lc) {

  float load_cell_values[LOAD_CELL_DOF] = {};
  float raw_load_cell_values[LOAD_CELL_DOF] = {};
  char style = 'c';
  int duration = 0;
  float time_delay = 0.002f;
  unsigned int marker = 1000;
  vector<double> load_vec = { 0, 0, 0, 0, 0, 0 };//added
  vector<vector<double>> loads;//added
  vector<vector<double>> loads1;//added
  char zero;

  //======== Initilize Load Cell ========//
  cout << "\nWould you like to zero load cell (y) or (n)?\n";
  cin >> zero;
  if (zero == 'y')
  {
	  cout << "\nDisconnect strips before zeroing load cell, press Any key and ENTER when ready\n";
	  char t;
	  cin >> t;
	  lc.Stop();
	  fprintf(stdout, "\nStarting load cell\n\n");
	  lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
		  Config::LOAD_CELL_TRANSFORMATION,
		  Config::SAMPLE_RATE,
		  Config::LOADCELL_CHANNEL);
	  delay(2000);
	  lc.SetBias();
	  delay(2000);
	  //initializing load cell code

	  //=========== Zero Load Cell ==========//
	  lc.GetLoads(load_cell_values);
	  while ((load_cell_values[0] > 0.1 || load_cell_values[0] < -0.1) ||
		  (load_cell_values[1] > 0.1 || load_cell_values[1] < -0.1) ||
		  (load_cell_values[2] > 0.1 || load_cell_values[2] < -0.1) ||
		  (load_cell_values[3] > 0.1 || load_cell_values[3] < -0.1) ||
		  (load_cell_values[4] > 0.1 || load_cell_values[4] < -0.1) ||
		  (load_cell_values[5] > 0.1 || load_cell_values[5] < -0.1))
	  {
		  delay(200);
		  lc.SetBias();
		  delay(200);
		  lc.GetLoads(load_cell_values);
	  }																										//sets bias of load cell, zeroing it after platform is at zero point
	  cout << "\nLoad Cell Zeroed\n";
	  delay(2000);
	  PrettyPrint::PrintLoad(load_cell_values);
  
  }

  fprintf(stdout, "Continous (c) or finite (f)?\n> ");
  cin >> style;
  switch (style) {
  case 'c':
    fprintf(stdout, "Time delay between samples (ms)?\n> ");
    cin >> time_delay;
    delay(2000);
    fprintf(stdout, "Hit ESC key to stop\n");

    while (1) {
      if (_kbhit()) {
        int stop_char = _getch();
        if (stop_char == 27) {
          break;
        }
		if (stop_char == 98)
		{
			lc.SetBias();
		}
        if (stop_char == 32) {
			lc.GetLoads(load_cell_values);
			for (int z = 0; z < 6; z++)//added
			{
				load_vec[z] = static_cast<double>(load_cell_values[z]);
			}
			loads1.push_back(load_vec);//added
        }
      }
      
      lc.GetLoads(load_cell_values);
      lc.GetRawLoads(raw_load_cell_values);
      PrettyPrint::PrintLoad(load_cell_values);
      delay(static_cast<DWORD>(time_delay));
	  for (int z = 0; z < 6; z++)//added
	  {
		  load_vec[z] = static_cast<double>(load_cell_values[z]);
	  }
	  loads.push_back(load_vec);//added
    }
    break;
  case 'f':
    fprintf(stdout, "How many to display (s)?\n> ");
    cin >> duration;
    fprintf(stdout, "Time delay between samples (ms)?\n> ");
    cin >> time_delay;
    for (float i = 0; i < duration; i += time_delay) {
      lc.GetLoads(load_cell_values);
      PrintLoad(load_cell_values);
      delay(static_cast<DWORD>(time_delay));
    }
    break;
  default:
    fprintf(stdout, "Wrong key %c", style);
    break;
  }

  //added
  int size = loads.size();
  string save_paths = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/loads/drifting.csv";
	ofstream savefile(save_paths);																//loops through each load file name to separate the loads
	for (int row = 0; row < size; row++)																//used to loop through all the vectors 
	{
	  for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		  {
			  savefile << loads[row][column];											//writes the specified element to the file
			  if (column != 5)
			  {
				  savefile << ", ";																		//writes comma after each value in the file
			  }
		  }
		  savefile << "\n";																				//writes a new line in the file onces a vector is completed
	  };
	int size1 = loads1.size();
	string save_paths1 = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/loads/selected_loads.csv";
	ofstream savefile1(save_paths1);																//loops through each load file name to separate the loads
	for (int row = 0; row < size1; row++)																//used to loop through all the vectors 
	{
		for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile1 << loads1[row][column];											//writes the specified element to the file
			if (column != 5)
			{
				savefile1 << ", ";																		//writes comma after each value in the file
			}
		}
		savefile1 << "\n";																				//writes a new line in the file onces a vector is completed
	};
}

void GetJointAnglesAndCartesianPose(LWR& lwr) {
  float joint_position[NUMBER_OF_JOINTS];
  lwr.GetCurrentJointPositions(joint_position);
  float cart_pose[NUMBER_OF_FRAME_ELEMENTS];
  lwr.GetCurrentCartesianPose(cart_pose);
  float path_point[19] = {
    joint_position[0], joint_position[1], joint_position[2], joint_position[3],
    joint_position[4], joint_position[5], joint_position[6],
    cart_pose[0], cart_pose[1], cart_pose[2], cart_pose[3],
    cart_pose[4], cart_pose[5], cart_pose[6], cart_pose[7],
    cart_pose[8], cart_pose[9], cart_pose[10], cart_pose[11] };
  Utils::ToClipboard(path_point, 19);
}
///////////////////////////////////////////////////////////////////////////
////                     FREE MOVEMENT & CONTROL MODES                 ////
///////////////////////////////////////////////////////////////////////////
void FreeCartesianMovement(LWR& lwr) {
  unsigned int movement_time;
  fprintf(stdout, "How many seconds would you like to free move for?");
  cin >> movement_time;
  FreeCartesianMovement(lwr, movement_time);
}

void FreeMovementPoseLoadRecord(LWR& lwr, Nano25E& lc) {
  static float FORCE_THRESHOLD[LOAD_CELL_DOF] = {
    0.05f, 0.05f, 0.05f, 0.002f, 0.002f, 0.002f
  };

  static float POSE_THRESHOLD[LOAD_CELL_DOF] = {
    0.005f, 0.005f, 0.005f, 10.0f, 10.0f, 10.0f
  };

  string log_filename;
  fprintf(stdout, "Enter log filename\n> ");
  cin >> log_filename;

  FreeMovementPoseLoadRecord(lwr, lc, LOGS_PATH + log_filename + CSV, true);
  ListPointsInLoadThreshold(LOGS_PATH + log_filename + CSV, FORCE_THRESHOLD,
    POSE_THRESHOLD, LOGS_PATH + log_filename + "_matches" + CSV);
}

void StartCartesianImpedanceControlMode(LWR& lwr) {
  lwr.StartCartesianImpedanceControlMode(
    lwr.CARTESIAN_STIFFNESS_HIGH,
    lwr.CARTESIAN_DAMPING_HIGH,
    lwr.CARTESIAN_TORQUE_NONE);
}

void StartPositionControlMode(LWR& lwr) {
  lwr.StartJointPositionControlMode(
    lwr.JOINT_STIFFNESS_HIGH,
    lwr.JOINT_DAMPING_HIGH,
    lwr.JOINT_TORQUE_NONE);
}

///////////////////////////////////////////////////////////////////////////
////                                 NAV                               ////
///////////////////////////////////////////////////////////////////////////
void NavByBendingForces(LWR& lwr, Nano25E& lc) {
  unsigned int POINT_DELAY = 0;
  string loads_follow_path;
  fprintf(stdout, "What LOADS path would you like to follow?\n> ");
  cin >> loads_follow_path;

  string output_loads_follow_path;
  fprintf(stdout, "Please name the output logged path file\n> ");
  cin >> output_loads_follow_path;
  string input_force_path = PATHS_LOAD_PATH + loads_follow_path + CSV;
  if (NavByBendingForces(lwr, lc, input_force_path,
    output_loads_follow_path, POINT_DELAY)) {
    fprintf(stdout, "Nav Failed\n");
    return;
  }
}

///////////////////////////////////////////////////////////////////////////
////                                MOVE                               ////
///////////////////////////////////////////////////////////////////////////
void MoveToJointPosition(LWR& lwr) {
  string position;
  vector<string> line_split;
  float joint_position[NUMBER_OF_JOINTS] = {};

  fprintf(stdout, "Enter joint angles (deg) or select a preset position:\n");
  for (auto const &it : PRESET_POSITIONS) {
    printf("%s\t", it.first.c_str());
  }
  PrintPrompt();
  cin >> position;
  transform(position.begin(), position.end(), position.begin(), ::tolower);
  if (PRESET_POSITIONS.count(position)) {
    lwr.MoveToJointPosition(reinterpret_cast<const float(&)[NUMBER_OF_JOINTS]>
      (PRESET_POSITIONS[position].second[0]), LWR::AngleUnit::RAD);
  }
  else {
    SplitLine(position, line_split, ",");
    if (line_split.size() != NUMBER_OF_JOINTS) {
      printf("Too many values or failed to comma delimit\n");
      Clear();
      return;
    }
    for (unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i) {
      joint_position[i] = static_cast<float>(atof(line_split.at(i).c_str()));
    }
    lwr.MoveToJointPosition(joint_position, LWR::AngleUnit::DEG);
  }
}

void MoveToCartesianPose(LWR& lwr, bool pid) {
  unsigned int transformation_selection;
  unsigned int err_val = SUCCESS;
  string position;
  vector<string> line_split;
  float cart_pose[NUMBER_OF_FRAME_ELEMENTS];

  fprintf(stdout, "Select a transformation option\n");
  for (auto const &it : pose_options) {
    printf("(%d) %s\n", it.first, it.second.c_str());
  }
  PrintPrompt();
  cin >> transformation_selection;
  switch (transformation_selection) {
  case TRANSFORMATION:
    fprintf(stdout, "Enter 12 element transformation matrix(4x3)");
    PrintPrompt();
    break;
  case TRANSLATION:
    fprintf(stdout, "Enter X,Y,Z translation(m)");
    PrintPrompt();
    break;
  case PRESET:
    fprintf(stdout, "Enter one of the preset poses\n");
    for (auto const &it : PRESET_POSITIONS) {
      printf("%s\t", it.first.c_str());
    }
    PrintPrompt();
    break;
  default:
    fprintf(stdout, "Incorrect option selected\n");
    Clear();
    err_val = ERROR_INVALID_SELECTION;
    break;
  }
  if (err_val != SUCCESS)
    return;

  cin >> position;
  SplitLine(position, line_split, ",");

  switch (transformation_selection) {
  case TRANSFORMATION:
    if (line_split.size() != NUMBER_OF_FRAME_ELEMENTS) {
      printf("12 elements required %d entered\n", line_split.size());
      err_val = ERROR_INCORRECT_NUM_ELEMENTS;
      break;
    }
    for (unsigned int i = 0; i < 12; ++i) {
      cart_pose[i] = static_cast<float>(atof(line_split.at(i).c_str()));
    }
    break;
  case TRANSLATION:
    if (line_split.size() != 3) {
      printf("3 elements required %d entered", line_split.size());
      err_val = ERROR_INCORRECT_NUM_ELEMENTS;
      break;
    }
    lwr.GetCurrentCartesianPose(cart_pose);
    cart_pose[3] = static_cast<float>(atof(line_split.at(0).c_str()));
    cart_pose[7] = static_cast<float>(atof(line_split.at(1).c_str()));
    cart_pose[11] = static_cast<float>(atof(line_split.at(2).c_str()));
    break;
  case PRESET:
    transform(position.begin(), position.end(), position.begin(), ::tolower);
    if (PRESET_POSITIONS.count(position)) {
      if (pid) {
        lwr.MoveToCartesianPose(
          reinterpret_cast<const float(&)[NUMBER_OF_FRAME_ELEMENTS]>
          (PRESET_POSITIONS[position].first[0]));
      }
      return;
    }
    else {
      printf("It's impressive that you managed to messed that up\n");
      err_val = ERROR_INCORRECT_NUM_ELEMENTS;
    }
    break;
  default:
    err_val = ERROR_INCORRECT_NUM_ELEMENTS;
    break;
  }
  if (err_val == SUCCESS) {
    lwr.MoveToCartesianPose(cart_pose);
  }
}

///////////////////////////////////////////////////////////////////////////////
////                                 STATS                                 ////
///////////////////////////////////////////////////////////////////////////////
void GeneratePathStats() {
  string results_file_base;
  unsigned int number_of_trials;
  unsigned int points_per_trial;

  fprintf(stdout, "Enter the results filename base?\n> ");
  cin >> results_file_base;
  fprintf(stdout, "How many path points does this path have?\n> ");
  cin >> points_per_trial;
  fprintf(stdout, "How many trials?\n> ");
  cin >> number_of_trials;

  Stats::GenerateNerdStatsFile(EXP_PATH + results_file_base, CSV,
    number_of_trials, points_per_trial);
}

///////////////////////////////////////////////////////////////////////////////
////                                 PLOTS                                 ////
///////////////////////////////////////////////////////////////////////////////
void GeneratePlots() {
  string input_file;
  string logged_file;
  fprintf(stdout,"Enter force path filename \n> ./path/loads/");
  cin >> input_file;
  fprintf(stdout, "\nEnter logged filename \n>  ./experiments/");
  cin >> logged_file;
  fprintf(stdout, "\n");
  GeneratePlots(PATHS_LOAD_PATH + input_file + CSV + "\"", EXP_PATH + logged_file + CSV + "\"");
}

// Generate the iPython plots by shelling out to command line and calling
// the plots CLI with the input and output filenames.
void GeneratePlots(string input, string output) {
  fprintf(stdout, "Generating plots\n");
  string cmd_ret = "";
  std::string cmd = Config::Filepath::python_path + " " + Config::Filepath::plot_gen + " -i \"" + input + " -o \"" + output ;
  cmd_ret = Utils::exec(cmd.c_str());
  std::cout << cmd_ret << "\n";
  fprintf(stdout, "Plots finished\n");
}

///////////////////////////////////////////////////////////////////////////////
////                                 DEBUG                                 ////
///////////////////////////////////////////////////////////////////////////////
void Debug(LWR& lwr, Nano25E & lc) 
{
  Debugging(lwr, lc);
}

void Clear() {
  cin.clear();
  cin.ignore(10000, '\n');
}
///////////////////////////////////////////////////////////////////////////////
////                               MHP OPEN                                ////
///////////////////////////////////////////////////////////////////////////////
void MHPOpen(IHexapodPtr& mhp, int robot)
{
	cout << "\nWhich robot are you using?\n1) Old Robot (MHP11_SN11007)\n2) New Robot (MHP-11L_SN11014)\n3) Skip Connecting Robot\n";

	_bstr_t rb;
	_bstr_t sn;

	while (1)
	{

		cin >> robot;
		switch (robot)
		{
			case 1:
				rb = L"MHP11_SN11007";				//Small Black Robot name
				sn = L"11007";						//Small Black Robot Serial Number
				break;
			case 2:
				rb = L"MHP-11L_SN11014";			//Large White Robot Name
				sn = L"11014";						//Large white Robot Serial Number
				break;
			case 3:
				return;			//skips this step
			default:
			{
				cout << "\nImproper selection, try again\n";
				robot = 0;
			}
		}
		break;
	}
	try
	{
		mhp->Open(rb);					//Opens the linked platform file from C:\ProgramData\Picard Industries\Hexapod\Devices\"...".mhp MHP11_SN11007
		mhp->Connect(VARIANT_FALSE, L"", sn);				//(VARIANT_"" (TRUE = simulate platform, FALSE = real platform), "address (port eg COM4)", "serial #") can leave address and serial # empty between ""
		mhp->SimulateMotionVelocity = VARIANT_TRUE;
		std::cout << "MHP Connected!\n";
	}
	catch (const _com_error& ex)
	{
		std::cout << "Could not Open/Connect\nTurn On Power!\n\n";
	}
	try
	{
		cout << "\n~~~~~ !!!!! ENSURE THE TOOL IS NOT ATTACHED CURRENTLY !!!!! ~~~~~~\n";
		cout << "\n~~~~~ !!!!!            Hit ENTER when safe            !!!!! ~~~~~~\n";
		int continue_value = 1;
		while (continue_value == 1)
		{
			if (_kbhit())
			{
				int stop_char = _getch();
				if (stop_char == 13)
				{
					continue_value = 0;
				}
			}
		}
		mhp->Home();											//Homes the MHP
		while (mhp->Moving)
		{
		}
		std::cout << "MHP Homed!\n\n";
	}
	catch (const _com_error& ex)
	{
		std::cout << "\nCould not Home!\n\n";
	}
}
///////////////////////////////////////////////////////////////////////////////
////                               MHP ZERO                                ////
///////////////////////////////////////////////////////////////////////////////
void MHPZero(IHexapodPtr& mhp, int robot)
{
	try
	{
		mhp->MovePosition(VARIANT_TRUE, L"Zero");				//Zeros MHP
		//mhp->MovePosition(VARIANT_TRUE, L"B2Zero");
		while (mhp->Moving)										//Loop to prevent any commands until robot is done moving
		{
		}
		MHPPrintCurrentPos(mhp, robot);
		std::cout << "MHP Zeroed!\n\n";
	}
	catch (const _com_error& ex)
	{
		std::cout << "\nCould not Zero\nMake Sure MHP Is Turned On and Has Been Connected!\n\n";
	}
}
///////////////////////////////////////////////////////////////////////////////
////                               MHP STOP                                ////
///////////////////////////////////////////////////////////////////////////////
void MHPStop(IHexapodPtr& mhp, int robot)
{
	try
	{
		mhp->Stop();				//Stops MHP
		std::cout << "MHP Stopped!\n\n";
	}
	catch (const _com_error& ex)
	{
		std::cout << "\nCould not Stop\nMHP May Not Be Connected!\n\n";
	}
}
///////////////////////////////////////////////////////////////////////////////
////                         MHP POSITION CONTROL                          ////
///////////////////////////////////////////////////////////////////////////////
void MHPPositionControl(IHexapodPtr& mhp, int robot)
{
	try
	{
		double x_int;
		double y_int;
		double z_int;
		char response = 'a';
		vector<double> pos_a = { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
		vector<double> pos_r = { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
		variant_t var_pos;
		variant_t var_pos_a;
		std::cout << "Would You like to move Absolute <a> or Relative <r>?\n";
		std::cin >> response;
		variant_t PosLimit_var = var_pos;								//creates a vector to hold positive limits of Px,Py,Pz,Mx,My,Mz
		variant_t NegLimit_var = var_pos;								//creates a vector to hold negative limits of Px,Py,Pz,Mx,My,Mz
		variant_t CurrentPos_var;
		variant_t* NegLimitPoint = &NegLimit_var;												//pointer for positive limits
		variant_t* PosLimitPoint = &PosLimit_var;												//pointer for negative limits
		vector<double> NegLimit;
		vector<double> PosLimit;
		switch (response)
		{
		case 'a':
			std::cout << "Coordinates are with respect to PLATFORM ZERO\n\n";
			std::cout << "Choose X position (mm): ";
			std::cin >> x_int;
			std::cout << "Choose Y position (mm): ";
			std::cin >> y_int;
			std::cout << "Choose Z position (mm): ";
			std::cin >> z_int;

			pos_a[12] = x_int;																	//sets X component of T-matrix
			pos_a[13] = y_int;																	//sets Y component of T-matrix
			pos_a[14] = z_int;																	//sets Z component of T-matrix

			var_pos = VectorToVariant(pos_a);													//converts vector to variant which can be used by MHP
			mhp->MoveMatrix(VARIANT_TRUE, L"Platform", L"Platform Zero", var_pos);				//uses v value to move the platform relative to the base by 1 mm in x

			std::cout << "Moving to: " << x_int << ", " << y_int << ", " << z_int << "\n\n";
			while (mhp->Moving)																	//Loop to prevent any commands until robot is done moving
			{
			}
			break;
		case 'r':
			std::cout << "Coordinates are relative to CURRENT POSITION\n\n";
			std::cout << "Choose X position (mm): ";
			std::cin >> x_int;
			std::cout << "Choose Y position (mm): ";
			std::cin >> y_int;
			std::cout << "Choose Z position (mm): ";
			std::cin >> z_int;

			pos_r[12] = x_int;																	//sets X component of T-matrix
			pos_r[13] = y_int;																	//sets Y component of T-matrix
			pos_r[14] = z_int;																	//sets Z component of T-matrix
			var_pos = VectorToVariant(pos_r);
			mhp->MoveMatrix(VARIANT_TRUE, L"Relative", L"Reamer", var_pos);
			std::cout << "Moving to: " << x_int << ", " << y_int << ", " << z_int << "\n\n";
			while (mhp->Moving)										//Loop to prevent any commands until robot is done moving
			{
			}
			break;
		default:
			fprintf(stdout, "Wrong key %c", response);
			break;
		}
	}
	catch (const _com_error& ex)
	{
		std::cout << "Error Could not move to position!\nMake Sure MHP Is Turned On and Has Been Connected!\n\n";
	}
}
///////////////////////////////////////////////////////////////////////////////
////				   25. MHP Run Path (Calibration)                      ////
///////////////////////////////////////////////////////////////////////////////

//Coordinate systems for the MHP can be changed by going to the C:\ProgramData\Picard Industries\Hexapod\Devices folder and altering "MHP11_SN11007" file (would recommend backing up original before altering)

void MHPRunPath(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
	try
	{
		float load_cell_values[LOAD_CELL_DOF] = {};																//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
		float load_cell_sum[LOAD_CELL_DOF] = {};																//variable that holds a sum total of loads when mulitple loads are taken in quick succession
		int length = 0;																							//stores the length (number of positions from the file given)

		//===== Connect and Zero MHP =====//
		char mhp1 = 'n';
		cout << "\nWould you like to connect Robot Platform (y) or (n)?\n";
		cin >> mhp1;
		cout << endl;
		if (mhp1 == 'y')
		{
			MHPOpen(mhp, robot);
		}
		MHPZero(mhp, robot);																							//moves mhp to zero position

		char zero = 'n';

		//======== Initilize Load Cell ========//
		cout << "\nWould you like to zero load cell (y) or (n)?\n";
		cin >> zero;
		if (zero == 'y')
		{
			cout << "\nDisconnect strips before zeroing load cell, press Any key and ENTER when ready\n";
			char t;
			cin >> t;
			lc.Stop();
			fprintf(stdout, "\nStarting load cell\n\n");
			lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
				Config::LOAD_CELL_TRANSFORMATION,
				Config::SAMPLE_RATE,
				Config::LOADCELL_CHANNEL);
			delay(2000);
			lc.SetBias();
			delay(2000);
			//initializing load cell code

			//=========== Zero Load Cell ==========//
			lc.GetLoads(load_cell_values);
			while ((load_cell_values[0] > 0.1 || load_cell_values[0] < -0.1) ||
				(load_cell_values[1] > 0.1 || load_cell_values[1] < -0.1) ||
				(load_cell_values[2] > 0.1 || load_cell_values[2] < -0.1) ||
				(load_cell_values[3] > 0.1 || load_cell_values[3] < -0.1) ||
				(load_cell_values[4] > 0.1 || load_cell_values[4] < -0.1) ||
				(load_cell_values[5] > 0.1 || load_cell_values[5] < -0.1))
			{
				delay(200);
				lc.SetBias();
				delay(200);
				lc.GetLoads(load_cell_values);
			}																										//sets bias of load cell, zeroing it after platform is at zero point
			cout << "\nLoad Cell Zeroed\n";
			PrettyPrint::PrintLoad(load_cell_values);
		}																									//initializing load cell code

		load_cell_sum[0] = 0;
		load_cell_sum[1] = 0;
		load_cell_sum[2] = 0;
		load_cell_sum[3] = 0;
		load_cell_sum[4] = 0;
		load_cell_sum[5] = 0;																										//intializes the sum variable
		int cycle = 1;																							//variable keeps track of which loop the MHP is on
		int cancel = 0;																							//variable to cancel moving loop if necessary
		string filestring;																						//user input path file name
		string savestring;																						//user input save file name
		string anglestring;																						//user input angles file name
		int loops = 0;																							//user input number of loops
		int load_loops = 20;																					//value that holds number of measurements are taken at each point
		cout << "\nWhat path would you like to follow?\n";
		cin >> filestring;
		cout << "\nHow many loops would you like to do?\n";
		cin >> loops;
		cout << "\nWhat file name would you like to save the loads to?\n";
		cin >> savestring;
		cout << "\n";

		//===== Set Velocity =====//
		MHPVelocity(mhp, lc);
		//this is still putting everything in the older folder rather than the folder with git, should probably be chaned
		string full_path = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/paths/New_Robot_Grid/" + filestring + ".csv";	//full path name of where the path file should be

		list<vector<double>> position;																			//declares a list of vectors to store all the position data
		vector<double> posvec_flip;																				//declares a vector that can store the value of a single position move
		vector<double> posvec_norm;																				//declares a vector that can store the value of a single position move
		vector<vector<double>> all_loads;																		//vector vectors to hold all the loads of the path
		vector<vector<vector<double>>> complete_loads;															//stores all the loads of all runs
		vector<double> loads_vec = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };												//vector that holds loads at a certain point ie Fx,Fy,Fz,Mx,My,Mz
		vector<double> intermediate = { 0, 0, 0, 0, 0, 0 };
		vector<vector<double>> intermediate_loads;
		int point_count = 1;

		/*//========= Digitize ==========//
		cout << "Do you want to Digitize the Scapula (y) or (n)?";
		char digitize = 'n';
		cin >> digitize;
		if (digitize == 'y')
		{
			MHPPIDLoadCell(mhp, lc, robot);
		}*/
		cout << "\n3..."; delay(1000); cout << "\n2..."; delay(1000); cout << "\n1..."; delay(1000);
		
		//========= Main Loop =========//
		for (; cycle <= loops && cancel == 0; cycle++)
		{
			//======= Move to Zero =======//
			MHPZero(mhp, robot);																						//Moves MHP to zero position
			while (mhp->Moving){}

			//======= Zero load Cell =======//
			/*cout << "Zeroing Load Cell...\n";
			lc.SetBias();
			lc.GetLoads(load_cell_values);
			while ((load_cell_values[0] > 0.1 || load_cell_values[0] < -0.1) ||
				(load_cell_values[1] > 0.1 || load_cell_values[1] < -0.1) ||
				(load_cell_values[2] > 0.1 || load_cell_values[2] < -0.1) ||
				(load_cell_values[3] > 0.1 || load_cell_values[3] < -0.1) ||
				(load_cell_values[4] > 0.1 || load_cell_values[4] < -0.1) ||
				(load_cell_values[5] > 0.1 || load_cell_values[5] < -0.1))
			{
				delay(200);
				lc.SetBias();
				delay(200);
				lc.GetLoads(load_cell_values);
			}																										//sets bias of load cell, zeroing it after platform is at zero point
			cout << "\nLoad Cell Zeroed\n";*/

			PrettyPrint::PrintLoad(load_cell_values);																									//Zeroes load cell
			ifstream pathfile(full_path);																		//path file that is used for below code
			if (pathfile.fail())
			{
				cout << "\nFile Name Does Not Exist\n\n";
				return;
			}
			cout << "================================================\n";
			cout << "Now on Loop: " << cycle << " of " << loops << "\n";
			string line;																						//variable to get the a line from the csv file
			int i = 0;																							//variable to control looping through the 16 values of each line
			while (getline(pathfile, line) && cancel == 0)														//gets a line from the csv as a string ex: "1,0,0,0,0,1,0,0,0,0,1,0,10,0,0,1"
			{
				stringstream ss(line);																			//
				while (ss.good() && cancel == 0)																//
				{
					string substr;																				//string variable to break apart the line of data, based on comma that separates each value
					getline(ss, substr, ',');																	//uses the line data gathered, and breaks apart each value
					double h = stod(substr);																	//converts string values to useable double values
					posvec_norm.push_back(h);																	//adds the value to the end of a vector
					i++;																						//increments a counting variable
					if (i == 16 && cancel == 0)
					{
						position.push_back(posvec_norm);														//adds the vector to the end of the list (once the vector has all 16 values)
						posvec_flip = MHPTranspose(posvec_norm);
						cout << "Moving to: " << posvec_flip[12] << ", " << posvec_flip[13] << ", " << posvec_flip[14] << "\n";
						cout << "Now on Point: " << point_count << " of " << length <<" , Loop: "<< cycle << " of " << loops << "\n";
						variant_t v = VectorToVariant(posvec_flip);												//converts vector to useable variable to move platform

						//====== Get Current X,Y,Z =======//
						variant_t pos_var1 = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
						vector<double> cur_pos1 = DoubleVariantToVector(pos_var1);								//currents 2D variant to a useable 1D 16 term vector
						vector<double> pos1 = MHPTranspose(cur_pos1);											//transposes matrix tp conventional T matrix 
						vector<double> del_xyz = { posvec_flip[12] - pos1[3], posvec_flip[13] - pos1[7], posvec_flip[14] - pos1[11] };


						mhp->MoveMatrix(VARIANT_TRUE, L"Reamer", L"Reamer Zero", v);							//uses v vector to move the platform relative to the base
						while ((del_xyz[0] > 0.1 || del_xyz[0] < -0.1 ||
							del_xyz[1] > 0.1 || del_xyz[1] < -0.1 ||
							del_xyz[2] > 0.1 || del_xyz[2] < -0.1) && cancel == 0)
						{
							pos_var1 = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");		//gets a variant that is 2D of current position
							cur_pos1 = DoubleVariantToVector(pos_var1);											//currents 2D variant to a useable 1D 16 term vector
							pos1 = MHPTranspose(cur_pos1);														//transposes matrix tp conventional T matrix 
							del_xyz = { posvec_flip[12] - pos1[3], posvec_flip[13] - pos1[7], posvec_flip[14] - pos1[11] };
							//cout << endl << del_xyz[0] << ", " << del_xyz[1] << ", " << del_xyz[2] << endl;

							if (_kbhit())
							{
								int stop_char = _getch();
								if (stop_char == 27)
								{
									mhp->Stop();
									cancel = 1;
									break;
								}																				//this two if statements check if escape is pressed, if it is it will stop the robot, basically and estop
							}																					//escape button press command
						}
						delay(300);																				//delay used to allow the load cell and MHP to settle reduce inertial effects in (milliseconds) 500
						for (int load_count = 1; load_count <= load_loops; load_count++)
						{
							lc.GetLoads(load_cell_values);														//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
							intermediate[0] = static_cast<double>(load_cell_values[0]);
							intermediate[1] = static_cast<double>(load_cell_values[1]);
							intermediate[2] = static_cast<double>(load_cell_values[2]);
							intermediate[3] = static_cast<double>(load_cell_values[3]);
							intermediate[4] = static_cast<double>(load_cell_values[4]);
							intermediate[5] = static_cast<double>(load_cell_values[5]);
							intermediate_loads.push_back(intermediate);
							PrettyPrint::PrintLoad(load_cell_values);
							loads_vec[0] = static_cast<double>(load_cell_values[0]) + loads_vec[0];
							loads_vec[1] = static_cast<double>(load_cell_values[1]) + loads_vec[1];
							loads_vec[2] = static_cast<double>(load_cell_values[2]) + loads_vec[2];
							loads_vec[3] = static_cast<double>(load_cell_values[3]) + loads_vec[3];
							loads_vec[4] = static_cast<double>(load_cell_values[4]) + loads_vec[4];
							loads_vec[5] = static_cast<double>(load_cell_values[5]) + loads_vec[5];				//changes variable type, easier to use
							delay(5);
						}
						intermediate = { 0, 0, 0, 0, 0, 0 };													//zeroes value
						intermediate_loads.push_back(intermediate);												//this adds in row of zeros between each set of values that were averaged to get the target point
						for (int z = 0; z <= 5; z++)
						{
							loads_vec[z] = loads_vec[z] / load_loops;											//converts the float variables to useable vector double
						}
						//======== Add XYZ end of Loads Vector ========//
						loads_vec[6] = posvec_flip[12];															//adds on the X value which is needed later for starting position in NavByBending Algorithm
						loads_vec[7] = posvec_flip[13];															//adds on the Y value which is needed later for starting position in NavByBending Algorithm
						loads_vec[8] = posvec_flip[14];															//adds on the Z value which is needed later for starting position in NavByBending Algorithm
						all_loads.push_back(loads_vec);															//adds the vector to the end of the list (once the vector has all 16 values)
						loads_vec = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
						posvec_norm.clear();																	//clears the position vector so it can store the next line on the next loop
						MHPPrintCurrentPos(mhp, robot);																//function that prints a matrix
						i = 0;																					//counter
					}
				}
				point_count++;																					//increases the point count once the previous point has been completed
			}
			cout << "\n";
			complete_loads.push_back(all_loads);																//stores a complete set of loads from an entire path
			length = all_loads.size();																			//gets the number of points per path
			all_loads.clear();																					//clears the loads 
			cout << "Complete Loop: " << cycle << " of " << loops << "\n";
			point_count = 1;
		}	
		vector<string> save_paths;
		string number;

			//========== Save Loads ==========//
			if (cancel != 1)
			{
				for (int k = 0; k < loops; k++)																			//creates seperate file names for each loop of loads
				{
					number = to_string(k);
					save_paths.push_back("C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/loads/New_Robot_Grid/" + savestring + "_" + number + ".csv"); //creates separate file names for each loop of loads
				}
				for (int section = 0; section < loops; section++)
				{
					ofstream savefile(save_paths[section]);																//loops through each load file name to separate the loads
					for (int row = 0; row < length; row++)																//used to loop through all the vectors 
					{
						for (int column = 0; column <= 8; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
						{
							savefile << complete_loads[section][row][column];											//writes the specified element to the file
							if (column != 8)
							{
								savefile << ", ";																		//writes comma after each value in the file
							}
						}
						savefile << "\n";																				//writes a new line in the file onces a vector is completed
					}
				}
				//========== Nerd Stats ==========//
				MHPNerdStats(complete_loads, length, loops, savestring);												//function to save nerd stats ie. mean and st dev
			}
			//==== Save Intermediate Loads ====//
			string save_paths1 = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/loads/New_Robot_Grid/" + savestring + "_intermediate.csv";
			ofstream savefile1(save_paths1);																			//loops through each load file name to separate the loads
			string header1 = "X,Y,Z,Rx,Ry,Rz\n";
			savefile1 << header1;
			int size1 = intermediate_loads.size();
			for (int row = 0; row < size1; row++)																	//used to loop through all the vectors 
			{
				for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
				{
					savefile1 << intermediate_loads[row][column];																//writes the specified element to the file
					savefile1 << ", ";																				//writes comma after each value in the file
				}
				savefile1 << "\n";																					//writes a new line in the file onces a vector is completed
			}
	}
	catch (const _com_error& ex)
	{
		std::cout << "Error! Could Not Run Path!\nMake Sure MHP Is Turned On and Has Been Connected!\n\n";
	}
}
///////////////////////////////////////////////////////////////////////////////
////                       MHP Nav By Bending Forces                       ////
///////////////////////////////////////////////////////////////////////////////
void MHPNavByBendingForces(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
		char mhp1 = 'n';
		cout << "\nWould you like to connect Robot Platform (y) or (n)?\n";
		cin >> mhp1;
		if (mhp1 == 'y')
		{
			MHPOpen(mhp, robot);
		}
		MHPZero(mhp, robot);																							//moves mhp to zero position
		float loads_msr[LOAD_CELL_DOF] = {};																	//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
		float loads_delta[LOAD_CELL_DOF] = {};																	//holds difference of desired and measured loads
		float loads_delta_converted[LOAD_CELL_DOF] = {};														//holds the delta loads after they have conversion factor applied
		char zero = 'n';
		while (mhp->Moving){};
		lc.GetLoads(loads_msr);
		delay(2000);
		PrettyPrint::PrintLoad(loads_msr);

		//======== Initilize Load Cell ========//
		cout << "\nWould you like to zero load cell (y) or (n)?\n";
		cin >> zero;
		if (zero == 'y')
		{
			cout << "\nDisconnect strips before zeroing load cell, press Any key and ENTER when ready\n";
			char t;
			cin >> t;
			lc.Stop();
			fprintf(stdout, "\nStarting load cell\n\n");
			lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
				Config::LOAD_CELL_TRANSFORMATION,
				Config::SAMPLE_RATE,
				Config::LOADCELL_CHANNEL);
			delay(2000);
			lc.SetBias();
			delay(2000);
			//initializing load cell code

			//=========== Zero Load Cell ==========//
			lc.GetLoads(loads_msr);
			while ((loads_msr[0] > 0.1 || loads_msr[0] < -0.1) ||
				(loads_msr[1] > 0.1 || loads_msr[1] < -0.1) ||
				(loads_msr[2] > 0.1 || loads_msr[2] < -0.1) ||
				(loads_msr[3] > 0.1 || loads_msr[3] < -0.1) ||
				(loads_msr[4] > 0.1 || loads_msr[4] < -0.1) ||
				(loads_msr[5] > 0.1 || loads_msr[5] < -0.1))
			{
				delay(200);
				lc.SetBias();
				delay(200);
				lc.GetLoads(loads_msr);
			}																										//sets bias of load cell, zeroing it after platform is at zero point
			cout << "\nLoad Cell Zeroed\n";
			delay(2000);
			PrettyPrint::PrintLoad(loads_msr);
		}
		Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;										//creates eigen matrix for current pose
		Eigen::Matrix<float, 4, 4, Eigen::RowMajor> final_pose_matrix;											//creates eigen matrix for result pose
		vector<vector<double>> lower_deadbands = MHPSetLowerDeadbands();										//creates vector of lower deadbands
		vector<vector<double>> upper_deadbands = MHPSetUpperDeadbands(mhp);										//creates vector of upper deadbands
		vector<vector<double>> limits = MHPSetLimits();															//creates vector of limit range, if inside these limits move on to next position
		vector<vector<double>> limits2 = MHPSetLimits2();															//creates vector of limit range, if inside these limits move on to next position
		vector<double> initial_pos_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };					//vector eventually used to as the starting position
		vector<double> next_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };						//creates vector for next pose
		string load_name;																						//variable to store file name that is to be loaded
		string save_name;																						//variable to store file name that is to be saved
		char hold = 'n';
		cout << "\nWhat load file do you wish to run?\n";
		cin >> load_name;
		cout << "\nWhat file name would you like to save to?\n";
		cin >> save_name;
		cout << "\nWould you like to hold position (y) or run path (n)?\n";										//variable to choose if you want to just hold a position (to see how robot reacts) or run path
		cin >> hold;
		if (hold != 'y' && hold != 'n')
		{
			hold = 'n';
		}
		mhp->PutVelocity(0.25);
		//MHPVelocity(mhp, lc);

		string load_file = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/loads/Nose_Load_Nav/" + load_name + ".csv";
		vector<vector<double>> loads_dsr;																		//vector that stores desired loads
		int load_loops = 20;																						//value that holds number of measurements are taken at each point
		vector<double> load_vec;																				//used to tranfer loads from a saved file being read to the loads_dsr vector of vectors
		vector<double> loads_sum = { 0, 0, 0, 0, 0, 0 };														//used to sum up a number of measured loads in quick succession
		vector<double> conversion_factor = MHPSetPControl();													//holds conversion factor for each load Fx,Fy,Fz,Mx,My,Mz
		int cancel = 0;
		variant_t pos = mhp->GetPositionMatrix(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");						//gets position of reamer with respect to reamer zero
		vector<double> pos_ = DoubleVariantToVector(pos);														//converts the get pos variant to vector, after the variant is converted from a array
		vector<double> current_pos_transpose = MHPTranspose(pos_);												//Transposes the matrix so it is in the conventional form
		vector<vector<double>> all_poses;																		//stores all values
		milliseconds timestamp;																					//timestamp variable
		vector<milliseconds> all_timestamps;																	//holds vector of timestamps
		vector<double> current_and_dsr = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };			//holds single line of the current T matrix and the desired position (ie. X,Y,Z)
		vector<vector<double>> all_msr_loads;																	//holds all the measured loads
		vector<double> vec_loads_delta = { 0, 0, 0, 0, 0, 0 };
		vector<double> vec_loads_delta_converted = { 0, 0, 0, 0, 0, 0 };
		vector<double> vec_u_limits = { 0, 0, 0, 0, 0, 0 };
		vector<double> vec_l_limits = { 0, 0, 0, 0, 0, 0 };
		vector<vector<double>> all_loads_delta;
		vector<vector<double>> all_loads_delta_converted;
		vector<vector<double>> all_u_limits;
		vector<vector<double>> all_l_limits;
		vector<double> limits_boolean = { 0, 0, 0, 0, 0, 0 };													//vector to hold boolean results of limit check
		vector<double> limits_boolean2 = { 0, 0, 0, 0, 0, 0 };													//vector to hold boolean results of limit check
		double limits_boolean_sum = 0;																			//vector to hold limit checks of each load cell component '0' for false '1' for true
		double limits_boolean_sum2 = 0;																			//vector to hold limit checks of each load cell component '0' for false '1' for true
		double limit_control_value = 4;																			//value that hold the number of limits that need to be true in order for the target point to be resolved
		double limit_control_value2 = 6;																		//value that hold the number of limits that need to be true in order for the target point to be resolved
		
		//======= Dot Product Variables =======//
		vector<double> point1 = { 0, 0, 0 };																	//holds point i-2 in otherwords a points for two ago
		vector<double> point2 = { 0, 0, 0 };																	//holds point i-1 in otherwords the previous point
		vector<double> point3 = { 0, 0, 0 };																	//holds point i   in otherwords the current point
		int loopcount = 0;																						//keeps count of while loop counter
		vector<double> Vi = { 0, 0, 0 };																		//initial vector between point 2 and point 1
		vector<double> Vf = { 0, 0, 0 };																		//initial vector between point 3 and point 2
		int dot_switch_count = 0;																				//keeps count of number of times dot product switches direction
		double dot_product = 0;																					//holds next dot product value

		//========== Reads Load File ==========//
		ifstream loads(load_file);
		if (loads.fail())
		{
			cout << "File name does not exist!\n\n";															//If the file doesn't exist the loop stops
			return;
		}
		string line;																							//variable to get the a line from the csv file
		int i = 0;																								//variable to control looping through the 16 values of each line
		while (getline(loads, line))																			//gets a line from the csv as a string ex: "1,0,0,0,0,1,0,0,0,0,1,0,10,0,0,1"
		{
			stringstream ss(line);																				//
			while (ss.good())																					//
			{
				string substr;																					//string variable to break apart the line of data, based on comma that separates each value
				getline(ss, substr, ',');																		//uses the line data gathered, and breaks apart each value
				double h = stod(substr);																		//converts string values to useable double values
				load_vec.push_back(h);																			//adds the value to the end of a vector
				i++;																							//increments a counting variable
				if (i == 9)
				{
					loads_dsr.push_back(load_vec);																//adds the vector to the end of the list (once the vector has all 16 values)
					load_vec.clear();																			//clears the position vector so it can store the next line on the next loop
					i = 0;																						//counter
				}
			}
		}																												//Gets Loads from a csv file

		//===== Move To Starting Position =====//
		int length = loads_dsr.size();																			//gets the number of rows in dsr load file to know number of loops
		initial_pos_vec[12] = loads_dsr[0][6];																	//sets the X value of the initial pose
		initial_pos_vec[13] = loads_dsr[0][7];																	//sets the Y value of the initial pose
		initial_pos_vec[14] = loads_dsr[0][8];																	//sets the Z value of the initial pose
		variant_t initial_pos_var = VectorToVariant(initial_pos_vec);
		delay(3000);
		mhp->MoveMatrix(VARIANT_TRUE, L"Reamer Nose", L"Reamer Nose Zero", initial_pos_var);								//moves mhp to initial position	
		while (mhp->Moving){};
		lc.GetLoads(loads_msr);																					//Gets initial loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_initial
		lc.GetLoads(loads_delta);																				//Initializes the delta loads variable, over-written later
		cout << "\n3..."; delay(1000); cout << "\n2..."; delay(1000); cout << "\n1..."; delay(1000);

		//======== Main Algorithm Loop ========//
		for (int row = 0; row < length; row++)
		{
			//===== Move to Zero/Zero Load Cell =====//
			cout << "\n=======================================================\n";
			cout << "\nNow on point: " << row + 1 << " of " << length <<"\n";
			while (1)
			{
				loopcount++;

				//========== Get Initial Loads ==========//
				for (int load_count = 1; load_count <= load_loops; load_count++)							//takes the average of a number of loops
				{
					lc.GetLoads(loads_msr);																	//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
					loads_sum[0] = static_cast<double>(loads_msr[0]) + loads_sum[0];
					loads_sum[1] = static_cast<double>(loads_msr[1]) + loads_sum[1];
					loads_sum[2] = static_cast<double>(loads_msr[2]) + loads_sum[2];
					loads_sum[3] = static_cast<double>(loads_msr[3]) + loads_sum[3];
					loads_sum[4] = static_cast<double>(loads_msr[4]) + loads_sum[4];
					loads_sum[5] = static_cast<double>(loads_msr[5]) + loads_sum[5];						//converts the float variables to useable vector double and sums up each individual load
					delay(5);
				}
				for (int z = 0; z <= 5; z++)
				{
					loads_sum[z] = loads_sum[z] / load_loops;												//averages each individual load
				}
				all_msr_loads.push_back(loads_sum);															//adds vector of the measured loads to the end of this variable
				all_msr_loads.push_back(loads_dsr[row]);													//adds vector of the desired loads to the end of this variable 

				//====== Dsr - Msr Load Difference ======//
				for (int term = 0; term < 6; term++)
				{
					loads_delta[term] = loads_sum[term] - loads_dsr[row][term];								//measured minus desired, since loads are difference is opposite to direction of travel
				}

				loads_sum = { 0, 0, 0, 0, 0, 0 };															//resets the load sum so that previous load values do not skew values

				//=========== Load Corrections ==========//
				for (int term = 0; term < 6; term++)
				{
					loads_delta_converted[term] = loads_delta[term] * conversion_factor[term];
				}

			//pose:																							//label used in the catch function to essential retry a failed step
				try																							//try and catch loop needed in order to prevent the program from stopping if a position can't be reached
				{

					//======== Set load to 0 if within limits ========//
					/*for (int i = 4; i <= 5; i++)
					{
						if (loads_delta[i] < limits[i][1] && loads_delta[i] > limits[i][0])
						{
							loads_delta_converted[i] = 0;
						}
					}*/

					//======== Check Upper Deadband =========//
					/*upper_deadbands = MHPSetUpperDeadbands(mhp);
					for (int term = 0; term < 6; term++)
					{
						if (loads_delta_converted[term] < 0
							&& loads_delta_converted[term] < upper_deadbands[term][0] * 0.05)										//checks if value is negative and > max neg deadband creating a threshold limit
						{
							loads_delta_converted[term] = upper_deadbands[term][0] * 0.05;											//sets value to a max step size, limits correction
						}
						if (loads_delta_converted[term] > 0
							&& loads_delta_converted[term] > upper_deadbands[term][1] * 0.05)										//checks if value is negative and > max neg deadband creating a threshold limit
						{
							loads_delta_converted[term] = upper_deadbands[term][1] * 0.05;											//sets value to a max step size, limits correction
						}
					}*/

					for (int term = 0; term < 6; term++)												
					{
						vec_loads_delta[term] = static_cast<double>(loads_delta[term]);
						vec_loads_delta_converted[term] = static_cast<double>(loads_delta_converted[term]);
						vec_u_limits[term] = static_cast<double>(upper_deadbands[term][1]);
						vec_l_limits[term] = static_cast<double>(upper_deadbands[term][0]);
					}
					all_loads_delta.push_back(vec_loads_delta);
					all_loads_delta_converted.push_back(vec_loads_delta_converted);
					all_u_limits.push_back(vec_u_limits);
					all_l_limits.push_back(vec_l_limits);

									
					//========= Gets Current Position From MHP Converts it =========//
					variant_t pos_var = mhp->GetPositionMatrix(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");		//gets a variant that is 2D of current position
					vector<double> cur_pos = DoubleVariantToVector(pos_var);										//currents 2D variant to a useable 1D 16 term vector
					cur_pos = MHPTranspose(cur_pos);																//transposes matrix vector to standard matrix form

					cur_pos[0] = 1;
					cur_pos[1] = 0;
					cur_pos[2] = 0;
					cur_pos[4] = 0;
					cur_pos[5] = 1;
					cur_pos[6] = 0;
					cur_pos[8] = 0;
					cur_pos[9] = 0;
					cur_pos[10] = 1;


					Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;
					current_pose_matrix <<
						cur_pos[0], cur_pos[1], cur_pos[2], cur_pos[3],    // [3] -> x
						cur_pos[4], cur_pos[5], cur_pos[6], cur_pos[7],    // [7] -> y
						cur_pos[8], cur_pos[9], cur_pos[10], cur_pos[11],  //[11] -> z
						cur_pos[12], cur_pos[13], cur_pos[14], cur_pos[15];									//turns vector into matrix for easy use of matrix multiplication function

					//====== Create Load Cell matrix =======//
					Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
					float sA = static_cast<float>(sin(RAD(loads_delta_converted[5])));						//Rz coverted to alpha angle
					float sB = static_cast<float>(sin(RAD(loads_delta_converted[4])));						//Ry coverted to beta angle
					float sG = static_cast<float>(sin(RAD(loads_delta_converted[3])));						//Rx coverted to gamma angle
					float cA = static_cast<float>(cos(RAD(loads_delta_converted[5])));						//Rz coverted to alpha angle
					float cB = static_cast<float>(cos(RAD(loads_delta_converted[4])));						//Ry coverted to beta angle
					float cG = static_cast<float>(cos(RAD(loads_delta_converted[3])));						//Rx coverted to gamma angle
					lc_matrix <<
						cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), loads_delta_converted[0],
						sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), loads_delta_converted[1],
						-sB, cB*sG, cB*cG, loads_delta_converted[2],
						0, 0, 0, 1;																			//load cell T matrix for small corrections

					//====== Matrix Multiplication To Base Coordinate System =====//
					final_pose_matrix = current_pose_matrix * lc_matrix;									//result pose calculation, mutliplcation of current and load cell T matrix

					//======== Convert Transform Calculation into Variant ========//
					for (unsigned int i = 0; i < 16; ++i) {
						next_pose_vec[i] = final_pose_matrix((i / 4), (i % 4));								//assign result pose back to vector
					}
					vector<double> next_pose_vec_trans = MHPTranspose(next_pose_vec);						//transposes vector so can be sent to MHP using its conventions

					/*next_pose_vec_trans[0] = 1;
					next_pose_vec_trans[1] = 0;
					next_pose_vec_trans[2] = 0;
					next_pose_vec_trans[4] = 0;
					next_pose_vec_trans[5] = 1;
					next_pose_vec_trans[6] = 0;
					next_pose_vec_trans[8] = 0;
					next_pose_vec_trans[9] = 0;
					next_pose_vec_trans[10] = 1;*/

					variant_t next_pose_var = VectorToVariant(next_pose_vec_trans);							//converts vector to variant to be sent to MHP

					//======= Moves platform/stores & displays current pose ======//
					mhp->MoveMatrix(VARIANT_TRUE, L"Reamer Nose", L"Reamer Nose Zero", next_pose_var);				//moves the platform based on the matrix sent to it in absolute terms

					//MHPPrintCurrentPos(mhp, robot);

					//=============== Dot Product ===============//
					  //consider using this to count if the robot is going back and forth over a point, if it does for more than 5 times move to next point
					/*point1 = point2;
					point2 = point3;
					point3 = { cur_pos[3], cur_pos[7], cur_pos[11] };
					if (loopcount > 2 && hold == 'n')
					{
						Vi = { point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2] };
						Vf = { point3[0] - point2[0], point3[1] - point2[1], point3[2] - point2[2] };
						dot_product = (Vf[0] * Vi[0]) + (Vf[1] * Vi[1]) + (Vf[2] * Vi[2]);
						if (dot_product < 0)
						{
							dot_switch_count++;
						}
						//cout << "\n" << dot_switch_count << "\n";
						if (dot_switch_count >= 5 && mhp->Moving)
						{
							break;
						}
					}*/

					//==================== Boolean Limit Check ===================//
					/*if (hold == 'y')
					{
						limits = MHPSetLowerDeadbands();
						for (int i = 0; i <= 5; i++)
						{
							if (loads_delta[i] < limits[i][1] && loads_delta[i] > limits[i][0])
							{
								loads_delta[i] = 0;
							}
						}
					}
					for (int i = 0; i <= 5; i++)
					{
						if (loads_delta[i] < limits[i][1] && loads_delta[i] > limits[i][0])
						{
							limits_boolean[i] = 1;		//value is true and within limits
						}
						else
						{
							limits_boolean[i] = 0;		//value is false and not within limits
						}
					}

					limits_boolean_sum = limits_boolean[0] + limits_boolean[1] + limits_boolean[2] + limits_boolean[3] + limits_boolean[4] + limits_boolean[5];
					//limits_boolean_sum2 = limits_boolean2[0] + limits_boolean2[1] + limits_boolean2[2] + limits_boolean2[3] + limits_boolean2[4] + limits_boolean2[5];

					//======= Check if adjusted load cell are within Limits ======//
					if (limits_boolean_sum >= limit_control_value && hold == 'n')
					{
						break;
					}*/

					//============== Load Check with in Limits ===============// 
					//If all loads within limits moves to next position
					if ((loads_delta[0] < limits[0][1] && loads_delta[0] > limits[0][0]) &&
						(loads_delta[1] < limits[1][1] && loads_delta[1] > limits[1][0]) &&
						(loads_delta[2] < limits[2][1] && loads_delta[2] > limits[2][0]) &&
						//(loads_delta[3] < limits[3][1] && loads_delta[3] > limits[3][0]) &&
						//(loads_delta[4] < limits[4][1] && loads_delta[4] > limits[4][0]) &&
						//(loads_delta[5] < limits[5][1] && loads_delta[5] > limits[5][0]) &&
						hold == 'n')
					{
						break;
					}

					//=========== Keyboard Hits ==========//
					if (_kbhit())
					{
						int stop_char = _getch();
						if (stop_char == 27)
						{
							mhp->Stop();
							goto LABEL;																		//exits program with ESC button press
					 	}																					
						if (stop_char == 32)
						{
							break;																			//skips to next position with space bar press
						}
					}																						//escape button press command
																				//prints current pose
					pos = mhp->GetPositionMatrix(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");				//gets position of platform with respect to platform zero
					pos_ = DoubleVariantToVector(pos);														//converts the get pos variant to vector, after the variant is converted from a array
					current_pos_transpose = MHPTranspose(pos_);												//Transposes the matrix so it is in the conventional form
					for (int r = 0; r <= 15; r++)
					{
						current_and_dsr[r] = current_pos_transpose[r];
					}
					current_and_dsr[16] = loads_dsr[row][6];
					current_and_dsr[17] = loads_dsr[row][7];
					current_and_dsr[18] = loads_dsr[row][8];
					all_poses.push_back(current_and_dsr);					

					//============== Get Timestamp and store vector ==============//
					timestamp = duration_cast< milliseconds >(
						system_clock::now().time_since_epoch());
					float time = timestamp.count();
					all_timestamps.push_back(timestamp);


				}
				catch (const _com_error& ex)
				{
					cout << "**************************  !!! OUT OF RANGE !!!  **************************\n\n";
					for (int term = 0; term < 6; term++)
					{
						loads_delta_converted[term] = loads_delta_converted[term] * 0.05;
					}
					if (_kbhit())
					{
						int stop_char = _getch();
						if (stop_char == 27)
						{
							mhp->Stop();
							goto LABEL;																		//exits program with ESC button press
						}
						if (stop_char == 32)
						{
							break;																			//skips to next position with space bar press
						}
					}
					/*cout << "\n" << loads_delta_converted[0] << ", " << loads_delta_converted[1] << ", " << loads_delta_converted[2] << ", " << loads_delta_converted[3] << ", " << loads_delta_converted[4] << ", " << loads_delta_converted[5] << "\n";					//}
					if (loads_delta_converted[0] < 0.05 && loads_delta_converted[1] < 0.05 && loads_delta_converted[2] < 0.05 && loads_delta_converted[3] < 0.05 && loads_delta_converted[4] < 0.05 && loads_delta_converted[5] < 0.05)
					{
						break;
					}
					for (int i = 0; i < 6; i++)
					{
						loads_delta_converted[i] = loads_delta_converted[i] * 0.5;
					}
					goto pose;*/															//skips to the try block after the previous error matrix has been halfed in order to skip the load cell re-measuring the loads
				}
			}
			loopcount = 0;
			dot_switch_count = 0;
		}

		//======= End of the Loop, Writing Results =======//
		LABEL:
		mhp->Stop();
		//======= Store and display final position =======//
		cout << "\nFinal ";
		MHPPrintCurrentPos(mhp, robot);
		cout << "Complete!\n\n";
		pos = mhp->GetPositionMatrix(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");								//gets position of platform with respect to platform zero
		pos_ = DoubleVariantToVector(pos);																		//converts the get pos variant to vector, after the variant is converted from a array
		current_pos_transpose = MHPTranspose(pos_);																//Transposes the matrix so it is in the conventional form
		for (int r = 0; r <= 15; r++)
		{
			current_and_dsr[r] = current_pos_transpose[r];
		}
		current_and_dsr[16] = loads_dsr[length-1][6];
		current_and_dsr[17] = loads_dsr[length-1][7];
		current_and_dsr[18] = loads_dsr[length-1][8];
		all_poses.push_back(current_and_dsr);

		//===== Get and store final load measurement =====//
		for (int load_count = 1; load_count <= load_loops; load_count++)
		{
			lc.GetLoads(loads_msr);																				//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
			loads_sum[0] = static_cast<double>(loads_msr[0]) + loads_sum[0];
			loads_sum[1] = static_cast<double>(loads_msr[1]) + loads_sum[1];
			loads_sum[2] = static_cast<double>(loads_msr[2]) + loads_sum[2];
			loads_sum[3] = static_cast<double>(loads_msr[3]) + loads_sum[3];
			loads_sum[4] = static_cast<double>(loads_msr[4]) + loads_sum[4];
			loads_sum[5] = static_cast<double>(loads_msr[5]) + loads_sum[5];
			delay(2);
		}
		for (int z = 0; z <= 5; z++)
		{
			loads_sum[z] = loads_sum[z] / load_loops;															//converts the float variables to useable vector double
		}
		all_msr_loads.push_back(loads_sum);
		all_msr_loads.push_back(loads_dsr[length-1]);

		//============ Store final timestamp =============//
		timestamp = duration_cast< milliseconds >(
			system_clock::now().time_since_epoch());
		float time = timestamp.count();
		all_timestamps.push_back(timestamp);
		timestamp = all_timestamps[1];
		int size = all_timestamps.size();

		//==== Saves results file of all current poses ====//
		string save_paths = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/results/Nose_Nav_Results/" + save_name +"_results.csv";
		ofstream savefile(save_paths);																			//loops through each load file name to separate the loads
		string header = "Time (ms),r00,r01,r02,X (mm),r10,r11,r12,Y (mm),r20,r21,r22,Z (mm),Row 0,Row 1,Row 2,Row 3,"
						"Dsr X (mm),Dsr Y (mm),Dsr Z (mm),Msr X (N),Dsr X (N),Msr Y (N),Dsr Y (N),Msr Z (N),Dsr Z (N),"
						"Msr Rx (Nm),Dsr Rx (Nm),Msr Ry (Nm),Dsr Ry (Nm),Msr Rz (Nm),Dsr Rz (Nm)\n";
		savefile << header;
		for (int row = 0; row < size; row++)																	//used to loop through all the vectors 
		{
			timestamp = all_timestamps[row] - all_timestamps[0];
			savefile << timestamp.count();
			savefile << ",";
			for (int column = 0; column <= 18; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
			{
				savefile << all_poses[row][column];																//writes the specified element to the file
				savefile << ", ";																				//writes comma after each value in the file
			}
			for (int column = 0; column <= 5; column++)
			{
				savefile << all_msr_loads[row*2][column];														//writes the specified element to the file
				savefile << ", ";																				//writes comma after each value in the file
				savefile << all_msr_loads[(row*2)+1][column];
				savefile << ", ";
			}
			savefile << "\n";																					//writes a new line in the file onces a vector is completed
		};

		//==== Save Limits and load differences ====//
		string save_paths1 = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/results/Nose_Nav_Results/" + save_name + "_results2.csv";
		ofstream savefile1(save_paths1);																				//loops through each load file name to separate the loads
		string header1 = "Delta X,Delta Y,Delta Z,Delta Rx,Delta Ry,Delta Rz,ConDel X,ConDel Y,ConDel Z,ConDel Rx,ConDel Ry,ConDel Rz,"
						"Lower X,Upper X,Lower Y,Upper Y,Lower Z,Upper Z,Lower Rx,Upper Rx,Lower Ry,Upper Ry,Lower Rz,Upper Rz\n";
		savefile1 << header1;
		int size1 = all_loads_delta.size();
		for (int row = 0; row < size1 -1; row++)																		//used to loop through all the vectors 
		{
			for (int column = 0; column <= 5; column++)																	//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
			{
				savefile1 << all_loads_delta[row][column];																//writes the specified element to the file
				savefile1 << ", ";																						//writes comma after each value in the file
			}
			for (int column = 0; column <= 5; column++)
			{
				savefile1 << all_loads_delta_converted[row][column];													//writes the specified element to the file
				savefile1 << ", ";																						//writes comma after each value in the file
			}
			for (int column = 0; column <= 5; column++)
			{
				savefile1 << all_l_limits[row][column];
				savefile1 << ", ";
				savefile1 << all_u_limits[row][column];
				savefile1 << ", ";
			}
			savefile1 << "\n";																					//writes a new line in the file onces a vector is completed
		};
}
///////////////////////////////////////////////////////////////////////////////
////						 MHP Joystick Digitize				 	       ////
///////////////////////////////////////////////////////////////////////////////
void MHPPIDLoadCell(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> final_pose_matrix;									//creates eigen matrix for result pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;								//creates eigen matrix for current pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> platform_zero_matrix;								//creates eigen matrix for platform zero

	float load_cell_values[LOAD_CELL_DOF] = {};														//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
	float load_cell_delta[LOAD_CELL_DOF] = {};
	float load_cell_zero[LOAD_CELL_DOF] = {};
	float load_cell_current[LOAD_CELL_DOF] = {};
	float load_cell_pid[LOAD_CELL_DOF] = {};
	vector<double> next_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };				//creates vector for next pose
	vector<double> current_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };			//creates vector for next pose
	variant_t CurrentPos_var = VectorToVariant(current_pose_vec);
	vector<double> load_vec = { 0, 0, 0, 0, 0, 0 };
	vector<vector<double>> loads1;
	vector<vector<double>> position;

	//======== P Controls ========//
	//p_x,  p_y,  p_z, p_rx, p_ry, p_rz
	vector<double> p_control = MHPSetPControl();													//proporital gain controller vector

	//====== Deadbands ======//
	vector<vector<double>> deadbands = { { -6, 6 }, { -4, 4 }, { -4, 4 }, { -4, 4 }, { -4, 4 }, { -4, 4 }, };	//set deadbands used for SENSITIVITY

	//=== Initialization ===//
	char mhp1 = 'n';
	cout << "\nWould you like to connect Robot Platform (y) or (n)?\n";
	cin >> mhp1;
	if (mhp1 == 'y')
	{
		MHPOpen(mhp, robot);
	}
	mhp1 = 'n';
	cout << "\nWould you like to Zero the Robot Platform (y) or (n)?\n";
	cin >> mhp1;
	if (mhp1 == 'y')
	{
		MHPZero(mhp, robot);
		while (mhp->Moving)
		{
		}
		cout << "\nRobot Zeroed\n";
	}

	//==== File Name ====//
	string file_name;
	cout << "\nWhat would you like to name the save file as?\n";
	cin >> file_name;

	float loads_msr[LOAD_CELL_DOF] = {};																	//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
	float loads_delta_converted[LOAD_CELL_DOF] = {};														//holds the delta loads after they have conversion factor applied
	char zero = 'n';
	while (mhp->Moving){};
	lc.GetLoads(loads_msr);
	delay(2000);
	vector<vector<double>> upper_deadbands = MHPSetUpperDeadbands(mhp);										//creates vector of upper deadbands

	//======== Initilize Load Cell ========//
	cout << "\nWould you like to connect load cell (y) or (n)?\n";
	cin >> zero;
	if (zero == 'y')
	{
		cout << "\nDisconnect strips before zeroing load cell, press ANY KEY and ENTER when ready\n";
		char t;
		cin >> t;
		lc.Stop();
		fprintf(stdout, "\nStarting load cell\n\n");
		lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
			Config::LOAD_CELL_TRANSFORMATION,
			Config::SAMPLE_RATE,
			Config::LOADCELL_CHANNEL);
		delay(2000);
		lc.SetBias();
		delay(2000);
		//initializing load cell code

		//=========== Zero Load Cell ==========//
		lc.GetLoads(loads_msr);
		while ((loads_msr[0] > 0.1 || loads_msr[0] < -0.1) ||
			(loads_msr[1] > 0.1 || loads_msr[1] < -0.1) ||
			(loads_msr[2] > 0.1 || loads_msr[2] < -0.1) ||
			(loads_msr[3] > 0.1 || loads_msr[3] < -0.1) ||
			(loads_msr[4] > 0.1 || loads_msr[4] < -0.1) ||
			(loads_msr[5] > 0.1 || loads_msr[5] < -0.1))
		{
			delay(200);
			lc.SetBias();
			delay(200);
			lc.GetLoads(loads_msr);
		}																										//sets bias of load cell, zeroing it after platform is at zero point
		cout << "\nLoad Cell Zeroed\n";
		delay(2000);
		PrettyPrint::PrintLoad(loads_msr);
		lc.GetLoads(load_cell_zero);
		lc.GetLoads(load_cell_values);
		lc.GetLoads(load_cell_pid);
		lc.GetLoads(load_cell_current);
		lc.GetLoads(load_cell_delta);																			//all of these initialize multiple load variables
	}

	//========== Set Software Limits ==========//
	/*
	vector<double> PosLim = { 33, 9.5, 10, 15, 15, 15 };														//Software Positive Position limits (mm), {x, y, z, Rx, Ry, Rz}
	vector<double> NegLim = { -33, -21, -20, -15, -15, -15 };													//Software Negative Position limits (mm), {x, y, z, Rx, Ry, Rz}
	variant_t PosLimit = VectorToVariant(PosLim);																//Convert vector to useable variant variable
	variant_t NegLimit = VectorToVariant(NegLim);																//Convert vector to useable variant variable
	_bstr_t Calibration;																						//Create variable for Limit variable
	mhp->CreateLimit(L"Calibration");																			//Creates Limit in hexapod database
	mhp->SetLimit(L"Calibration", L"Reamer", L"Reamer Zero", &NegLimit, &PosLimit);								//Fills the Created Limit with dimensions of limits {L"Name of Limit Created", Object CS, Reference CS, Negative limits, Positive Limits)
	*/  //remember to reinclude the deletelimie(Calibration) 

	//============== Set  Velocity ==============//
	MHPVelocity(mhp, lc);

	//============ Reconnect Strips ============//
	cout << "\nReconnect Strips and position robot in correct position.\nPress ANY KEY and ENTER when ready\n";
	char t;
	cin >> t;																									//input just to progress to next steps when ready

	cout << "\nMeasuring current loads\n";
	lc.GetLoads(load_cell_zero);
	cout << "\nLoads Recorded\n";

	//========== Commands During Code ==========//
	cout << "Press 'Esc' to get out of loop\n";
	cout << "Press 'Enter' to hold position\n";
	cout << "Press 'Spacebar' to record loads and position\n";
	cout << "Press 'p' for current robot position\n";
	cout << "Press 'l' for current load cell values\n";
	cout << "Press 'z' to zero robot\n\n";

	PrettyPrint::PrintLoad(load_cell_values);												//function to print load values on screen

	/////////////////////// LOOP ///////////////////////
	while (1)
	{
		try																							// try/catch need to be inside while loop with the movement function in order to catch any positions that are...
		{																							//...outside the range and would normally cause an error and stop the program
			//========= Gets Current Position From MHP Converts it =========//
			variant_t pos_var = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
			vector<double> cur_pos = DoubleVariantToVector(pos_var);								//currents 2D variant to a useable 1D 16 term vector
			vector<double> pos_ = MHPTranspose(cur_pos);											//converts variant from Left handed to right handed coordinate system, need to send variable and current loads
			current_pose_matrix <<																	//switches normal T matrix to eigen matrix to be able to use matrix functions (ie. matrix multiplication)
				pos_[0], pos_[1], pos_[2], pos_[3],
				pos_[4], pos_[5], pos_[6], pos_[7],
				pos_[8], pos_[9], pos_[10], pos_[11],
				0, 0, 0, 1;
			if (_kbhit())
			{
				int stop_char = _getch();
				if (stop_char == 112)
				{
					MHPPrintCurrentPos(mhp, robot);
				}
				if (stop_char == 108)
				{
					lc.GetLoads(load_cell_values);
					PrettyPrint::PrintLoad(load_cell_values);
				}
				if (stop_char == 122)																//'z' push
				{
					vector<double> movebackvec = { 0, 0, -5, 0, 0, 0 };
					variant_t moveback = VectorToVariant(movebackvec);
					mhp->MoveTuple(VARIANT_TRUE, L"Relative", L"Reamer", moveback);
					while (mhp->Moving)																//Loop to prevent any commands until robot is done moving
					{
					}
					mhp->MovePosition(VARIANT_TRUE, L"Zero");										//Zeros MHP
					while (mhp->Moving)																//Loop to prevent any commands until robot is done moving
					{
					}
					std::cout << "MHP Zeroed!\n\n";
					lc.GetLoads(load_cell_values);
					PrettyPrint::PrintLoad(load_cell_values);
				}
				if (stop_char == 27)																//Esc command to get out of loop
				{
					mhp->Stop();
					cout << "\nLoop Stopped\n\n";
					break;
				}																					//this two if statements check if escape is pressed, if it is it will stop the robot, basically and estop
				if (stop_char == 32)																//'Spacebar' push
				{
					lc.GetLoads(load_cell_values);
					for (int z = 0; z < 6; z++)//added
					{
						load_vec[z] = static_cast<double>(load_cell_values[z]);
					}
					variant_t pos_var1 = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
					vector<double> cur_pos1 = DoubleVariantToVector(pos_var1);								//currents 2D variant to a useable 1D 16 term vector
					vector<double> pos1 = MHPTranspose(cur_pos1);											//transposes matrix tp conventional T matrix 
					vector<double> xyz = { pos1[3], pos1[7], pos1[11] };
					loads1.push_back(load_vec);//added
					position.push_back(xyz);
					cout << "\n";
					PrettyPrint::PrintLoad(load_cell_values);												//function to print load values on screen
					MHPPrintCurrentPos(mhp, robot);
					cout << "Press 'Esc' to get out of loop\n";
					cout << "Press 'Enter' to hold position\n";
					cout << "Press 'Spacebar' to record loads and position\n";
					cout << "Press 'p' for current robot position\n";
					cout << "Press 'l' for current load cell values\n";
					cout << "Press 'z' to zero robot\n\n";
				}
				if (stop_char == 13)																//Enter key is hit
				{
					mhp->Stop();
					cout << "|||||~ ~ ~ ~ ~ ~ ~ HOLDING POSITION ~ ~ ~ ~ ~ ~ ~|||||";
					cout << "Press 'Esc' to get out of loop\n";
					cout << "Press 'Spacebar' to record loads and position\n";
					cout << "Press 'p' for current robot position\n";
					cout << "Press 'l' for current load cell values\n";
					cout << "Press 'z' to zero robot\n\n";

					while (1)
					{
						stop_char = _getch();
						if (stop_char == 27)														//Esc command to exit loop
						{
							cout << "\n|||||~ ~ ~ ~ ~ ~ ~ HOLD POSITION STOPPED ~ ~ ~ ~ ~ ~ ~|||||\n\n";
							cout << "\n=============================================\n";
							cout << "Press 'Esc' to get out of loop\n";
							cout << "Press 'Enter' to hold position\n";
							cout << "Press 'Spacebar' to record loads and position\n";
							cout << "Press 'p' for current robot position\n";
							cout << "Press 'l' for current load cell values\n";
							cout << "Press 'z' to zero robot\n\n";
							break;
						}
						if (stop_char == 32)														//Spacebar is hit
						{
							lc.GetLoads(load_cell_values);
							for (int z = 0; z < 6; z++)//added
							{
								load_vec[z] = static_cast<double>(load_cell_values[z]);
							}
							variant_t pos_var1 = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
							vector<double> cur_pos1 = DoubleVariantToVector(pos_var1);								//currents 2D variant to a useable 1D 16 term vector
							vector<double> pos1 = MHPTranspose(cur_pos1);											//transposes matrix tp conventional T matrix 
							vector<double> xyz = { pos1[3], pos1[7], pos1[11] };
							loads1.push_back(load_vec);//added
							position.push_back(xyz);
							cout << "\n";
							PrettyPrint::PrintLoad(load_cell_values);												//function to print load values on screen
							MHPPrintCurrentPos(mhp, robot);
							cout << "Press 'Esc' to get out of loop\n";
							cout << "Press 'Spacebar' to record loads and position\n";
							cout << "Press 'p' for current robot position\n";
							cout << "Press 'l' for current load cell values\n";
							cout << "Press 'z' to zero robot\n\n";
						}
						if (stop_char == 122)														//'z' push
						{
							mhp->MovePosition(VARIANT_TRUE, L"Zero");								//Zeros MHP
							while (mhp->Moving)														//Loop to prevent any commands until robot is done moving
							{
							}
							std::cout << "MHP Zeroed!\n\n";
							lc.GetLoads(load_cell_values);
							PrettyPrint::PrintLoad(load_cell_values);
						}
						if (stop_char == 112)
						{
							MHPPrintCurrentPos(mhp, robot);
						}
						if (stop_char == 108)
						{
							lc.GetLoads(load_cell_values);
							PrettyPrint::PrintLoad(load_cell_values);
						}
					}
				}
			}	

			lc.GetLoads(load_cell_values);
			for (int z = 0; z < 6; z++)//added
			{
				load_vec[z] = static_cast<double>(load_cell_values[z]);
			}
			variant_t pos_var1 = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
			vector<double> cur_pos1 = DoubleVariantToVector(pos_var1);								//currents 2D variant to a useable 1D 16 term vector
			vector<double> pos1 = MHPTranspose(cur_pos1);											//transposes matrix tp conventional T matrix 
			vector<double> xyz = { pos1[3], pos1[7], pos1[11] };
			loads1.push_back(load_vec);//added
			position.push_back(xyz);

			//========== Loads Difference and Proportial Control ==========//
			lc.GetLoads(load_cell_values);															//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
			for (int z = 0; z < 6; z++)
			{
				load_cell_delta[z] = load_cell_values[z] - load_cell_zero[z];						//calculates a delta loads from the initial zero
			}

			load_cell_delta[3] = 0;
			load_cell_delta[4] = 0;
			load_cell_delta[5] = 0;																	//zeros torques so that only translations and no rotations occur

			for (int d = 0; d < 6; d++)
			{
				if (load_cell_delta[d] > deadbands[d][0] && load_cell_delta[d] < deadbands[d][1])	//checks if the adjusted delta load cell values are within the deadband values
				{
					load_cell_delta[d] = 0;															//sets value to zero within a range to prevent jittering
				}
			}
			for (int p = 0; p < 6; p++)
			{
				load_cell_pid[p] = load_cell_delta[p];// *p_control[p];								//proportial controller
			}

			//================ Load Cell Transform Matrix ================//
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
			float sA = static_cast<float>(sin(RAD(load_cell_pid[5])));								//Rz coverted to alpha angle
			float sB = static_cast<float>(sin(RAD(load_cell_pid[4])));								//Ry coverted to beta angle
			float sG = static_cast<float>(sin(RAD(load_cell_pid[3])));								//Rx coverted to gamma angle
			float cA = static_cast<float>(cos(RAD(load_cell_pid[5])));								//Rz coverted to alpha angle
			float cB = static_cast<float>(cos(RAD(load_cell_pid[4])));								//Ry coverted to beta angle
			float cG = static_cast<float>(cos(RAD(load_cell_pid[3])));								//Rx coverted to gamma angle
			/*lc_matrix <<
				cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), load_cell_pid[0],
				sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), load_cell_pid[1],
				-sB, cB*sG, cB*cG,                                 load_cell_pid[2],
				0, 0, 0, 1;																			//load cell T matrix*/

			lc_matrix <<
				1, 0, 0, load_cell_pid[0],
				0, 1, 0, load_cell_pid[1],
				0, 0, 1, load_cell_pid[2],
				0, 0, 0, 1;
			//====== Matrix Multiplication To Base Coordinate System =====//
			final_pose_matrix = current_pose_matrix * lc_matrix;									//result pose calculation, mutliplcation of current and load cell T matrix
			
			//======== Convert Transform Calculation into Variant ========//
			for (unsigned int i = 0; i < 16; ++i) {
				next_pose_vec[i] = final_pose_matrix((i / 4), (i % 4));								//assign result pose back to vector
			}
			vector<double> next_pose_vec_trans = MHPTranspose(next_pose_vec);						//transposes vector so can be sent to MHP using its conventions
			variant_t next_pose_var = VectorToVariant(next_pose_vec_trans);							//converts vector to variant to be sent to MHP

			//======= Check if adjusted load cell are within Limits ======//																						//checks if any of the controls are non-zero
			if (load_cell_pid[0] != 0 || load_cell_pid[1] != 0 || load_cell_pid[2] != 0) // ||
				//load_cell_pid[3] != 0 || load_cell_pid[4] != 0 || load_cell_pid[5] != 0)
			{
				mhp->MoveMatrix(VARIANT_TRUE, L"Platform", L"Platform Zero", next_pose_var);		//moves the platform based on the matrix sent to it
				PrettyPrint::PrintLoad(load_cell_values);												//function to print load values on screen
			}
			else if (mhp->Moving)
			{
				mhp->Stop();
				PrettyPrint::PrintLoad(load_cell_values);												//function to print load values on screen
			}
		}
		catch (const _com_error& ex)
		{
			std::cout << "Position is Out of Range!\n";
			mhp->Stop();
		}
	}

	//mhp->DeleteLimit(L"Calibration");

	int size1 = loads1.size();
	string save_paths1 = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/digitize/" + file_name + "_coordinates.csv";
	ofstream savefile1(save_paths1);																//loops through each load file name to separate the loads
	for (int row = 0; row < size1; row++)																//used to loop through all the vectors 
	{
		for (int column = 0; column <= 2; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile1 << position[row][column];											//writes the specified element to the file
			if (column != 2)
			{
				savefile1 << ", ";																		//writes comma after each value in the file
			}
		}
		savefile1 << "\n";																				//writes a new line in the file onces a vector is completed
	};
	string save_loads = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/digitize/" + file_name + "_loads.csv";
	ofstream savefile2(save_loads);																//loops through each load file name to separate the loads
	for (int row = 0; row < size1; row++)																//used to loop through all the vectors 
	{
		for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile2 << loads1[row][column];											//writes the specified element to the file
			if (column != 5)
			{
				savefile2 << ", ";																		//writes comma after each value in the file
			}
		}
		savefile2 << "\n";																				//writes a new line in the file onces a vector is completed
	};
}
///////////////////////////////////////////////////////////////////////////////
////					 MHP Manual Input Loads Control				       ////
///////////////////////////////////////////////////////////////////////////////
void MHPManualLoadCell(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
	MHPOpen(mhp, robot);																					//homes MHP if not done already
	MHPZero(mhp, robot);																					//zeros MHP
	vector<double> pose_vec_flip = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };				//this is the initial T-matrix of platform wrt platform zero (they are equal)
	vector<double> pose_vec_norm = TFlipToTNorm(pose_vec_flip);										//flips initial T to normal T matrix
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> result_pose_matrix;									//creates eigen matrix for result pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> final_pose_matrix;									//creates eigen matrix for result pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;								//creates eigen matrix for current pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> platform_zero_matrix;								//creates eigen matrix for platform zero

	float load_cell_values[LOAD_CELL_DOF] = {};														//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
	float load_cell_pid[LOAD_CELL_DOF] = {};
	float load_cell_initial[LOAD_CELL_DOF] = {};
	float load_cell_delta[LOAD_CELL_DOF] = {};
	float load_cell_zero[LOAD_CELL_DOF] = {};
	vector<double> next_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };				//creates vector for next pose
	vector<double> current_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };			//creates vector for next pose
	variant_t CurrentPos_var = VectorToVariant(current_pose_vec);

	vector<double> p_control = MHPSetPControl();													//proporital controller vector
	vector<vector<double>> deadbands = MHPSetLowerDeadbands();											//Sets the Deadbands

	current_pose_matrix <<																			//switches normal T matrix to eigen matrix to be able to use matrix functions (ie. matrix multiplication)
		pose_vec_norm[0], pose_vec_norm[1], pose_vec_norm[2], pose_vec_norm[3],
		pose_vec_norm[4], pose_vec_norm[5], pose_vec_norm[6], pose_vec_norm[7],
		pose_vec_norm[8], pose_vec_norm[9], pose_vec_norm[10], pose_vec_norm[11],
		0, 0, 0, 1;
	platform_zero_matrix << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;							//populates matrix of platform_zero_matrix

		if (lc.GetLoads(load_cell_values) != 0)															//checks if load cell is initialized, if not it initializes it
	{
		fprintf(stdout, "Starting load cell\n\n");
		lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
			Config::LOAD_CELL_TRANSFORMATION,
			Config::SAMPLE_RATE,
			Config::LOADCELL_CHANNEL);
		delay(2000);
		lc.SetBias();
		delay(2000);
	}																								//load cell initialization
	lc.GetLoads(load_cell_zero);
	lc.GetLoads(load_cell_delta);
	lc.GetLoads(load_cell_pid);																								//Load Cell initialize code
	cout << "Press Esc to get out of loop\n";

	/////////////////////// LOOP ///////////////////////
	while (1)
	{
		try
		{
			if (_kbhit())
			{
				int stop_char = _getch();
				if (stop_char == 27)
				{
					mhp->Stop();
					break;
				}																					//this two if statements check if escape is pressed, if it is it will stop the robot, basically and estop
			}																							//Esc command to get out of loop
			if (_kbhit())
			{
				int stop_char = _getch();
				if (stop_char == 32)
				{
					mhp->MovePosition(VARIANT_TRUE, L"Zero");				//Zeros MHP
					while (mhp->Moving)										//Loop to prevent any commands until robot is done moving
					{
					}
				}																					//this two if statements check if escape is pressed, if it is it will stop the robot, basically and estop
			}																						//Use spacebar to return to zero point

			cout << "Fx [N]: "; cin >> load_cell_pid[0];
			cout << "Fy [N]: "; cin >> load_cell_pid[1];
			cout << "Fz [N]: "; cin >> load_cell_pid[2];
			cout << "Tx [N]: "; cin >> load_cell_pid[3];
			cout << "Ty [N]: "; cin >> load_cell_pid[4];
			cout << "Tz [N]: "; cin >> load_cell_pid[5];
			cout << "\n";

			for (int p = 0; p < 6; p++)
			{
				load_cell_pid[p] = load_cell_pid[p] * p_control[p];									//proportial controller
			}
			for (int d = 0; d < 6; d++)
			{
				if (load_cell_pid[d] > 10)															//checks if the load cell values are too high
				{
					load_cell_pid[d] = 10;															//limits load cell value to maximum 5
				}
			}
			for (int d = 0; d < 6; d++)
			{
				if (load_cell_pid[d] < -10)															//checks if the load cell values are too low
				{
					load_cell_pid[d] = -10;															//limits load cell value to maximum-5
				}
			}
			//================ Load Cell Transform Matrix ================//
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
			float sA = static_cast<float>(sin(RAD(load_cell_pid[5])));								//Rz coverted to alpha angle
			float sB = static_cast<float>(sin(RAD(load_cell_pid[4])));								//Ry coverted to beta angle
			float sG = static_cast<float>(sin(RAD(load_cell_pid[3])));								//Rx coverted to gamma angle
			float cA = static_cast<float>(cos(RAD(load_cell_pid[5])));								//Rz coverted to alpha angle
			float cB = static_cast<float>(cos(RAD(load_cell_pid[4])));								//Ry coverted to beta angle
			float cG = static_cast<float>(cos(RAD(load_cell_pid[3])));								//Rx coverted to gamma angle
			lc_matrix <<
				cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), load_cell_pid[0],
				sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), load_cell_pid[1],
				-sB, cB*sG, cB*cG, load_cell_pid[2],
				0, 0, 0, 1;																			//load cell T matrix
			//====== Matrix Multiplication To Base Coordinate System =====//
			result_pose_matrix = current_pose_matrix * lc_matrix;									//result pose calculation, mutliplcation of current and load cell T matrix
			final_pose_matrix = platform_zero_matrix * result_pose_matrix;							//T matrix of tool relative to platform zero
			//======== Convert Transform Calculation into Variant ========//
			for (unsigned int i = 0; i < 12; ++i) {
				next_pose_vec[i] = final_pose_matrix((i / 4), (i % 4));								//assign result pose back to vector
			}
			vector<double> next_pose_vec_flip = TNormToTFlip(next_pose_vec);						//flips last column with last row due to how MHP T-matrix is defined see TNormToTFlip function
			variant_t next_pose_var = VectorToVariant(next_pose_vec_flip);							//converts the flip matrix to variant that the MHP needs
			next_pose_var = RightToLeft(next_pose_var,load_cell_pid);								//converts from a right hand to left hand coordinate system

			//======= Check if adjusted load cell are within Limits ======//
			cout << "Loads: ";
			PrettyPrint::PrintLoad(load_cell_pid);													//prints loads being sent, if needed to be seen
			cout << "\n";
			if (load_cell_pid[0] != 0 || load_cell_pid[1] != 0 || load_cell_pid[2] != 0 || load_cell_pid[3] != 0 || load_cell_pid[4] != 0 || load_cell_pid[5] != 0) 
			{																						//checks if any of the controls are non-zero
				mhp->MoveMatrix(VARIANT_TRUE, L"Relative", L"Platform", next_pose_var);				//moves the platform based on the matrix sent to it
				while (mhp->Moving)
				{
				}
				MHPPrintCurrentPos(mhp, robot);
			}
			else
			{
				if (mhp->Moving)																	//checks if the platform is moving
				{
					mhp->Stop();																	//stops platform, if the load cell values are 0 meaning its not being pressed
				}
				mhp->MovePosition(VARIANT_TRUE, L"Zero");											//Zeros MHP
			}
		}
		catch (const _com_error& ex)
		{
			std::cout << "Position is Out of Range!\n";
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
////							MHP Move Back					           ////
///////////////////////////////////////////////////////////////////////////////
void MHPMoveBack(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
	vector<double> pos = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -8, 0, 0, 0, 1 };
	vector<double> posT = MHPTranspose(pos);
	variant_t pos_var = VectorToVariant(posT);
	mhp->MoveMatrix(VARIANT_TRUE, L"Platform", L"Platform Zero", pos_var);
}

///////////////////////////////////////////////////////////////////////////////
////					       MHP CNC Code Ear					           ////
///////////////////////////////////////////////////////////////////////////////
void MHPcnc(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
	milliseconds timestamp;																						//timestamp variable
	vector<milliseconds> all_timestamps;																		//holds vector of timestamps
	string save_name;																							//save name variable creator
	vector<vector<double>> all_poses;																			//stores all values
	vector<double> current_loads = { 0, 0, 0, 0, 0, 0 };
	vector<vector<double>> all_loads;																		//vector vectors to hold all the loads of the path
	variant_t var_pos;

	try
	{
		float load_cell_values[LOAD_CELL_DOF] = {};																//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
		float load_cell_sum[LOAD_CELL_DOF] = {};																//variable that holds a sum total of loads when mulitple loads are taken in quick succession
		int length = 0;																							//stores the length (number of positions from the file given)

		//===== Connect and Zero MHP =====//
		char mhp1 = 'n';
		cout << "\nWould you like to connect Robot Platform (y) or (n)?\n";
		cin >> mhp1;
		cout << endl;
		if (mhp1 == 'y')
		{
			MHPOpen(mhp, robot);
		}
		MHPZero(mhp, robot);																							//moves mhp to zero position

		//======== Initilize Load Cell ========//
		cout << "\nWould you like to zero load cell (y) or (n)?\n";
		char zero = 'n';
		cin >> zero;
		if (zero == 'y')
		{
			cout << "\nDisconnect strips before zeroing load cell, press Any key and ENTER when ready\n";
			char t;
			cin >> t;
			lc.Stop();
			fprintf(stdout, "\nStarting load cell\n\n");
			lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
				Config::LOAD_CELL_TRANSFORMATION,
				Config::SAMPLE_RATE,
				Config::LOADCELL_CHANNEL);
			delay(2000);
			lc.SetBias();
			delay(2000);
			//initializing load cell code

			//=========== Zero Load Cell ==========//
			lc.GetLoads(load_cell_values);
			while ((load_cell_values[0] > 0.1 || load_cell_values[0] < -0.1) ||
				(load_cell_values[1] > 0.1 || load_cell_values[1] < -0.1) ||
				(load_cell_values[2] > 0.1 || load_cell_values[2] < -0.1) ||
				(load_cell_values[3] > 0.1 || load_cell_values[3] < -0.1) ||
				(load_cell_values[4] > 0.1 || load_cell_values[4] < -0.1) ||
				(load_cell_values[5] > 0.1 || load_cell_values[5] < -0.1))
			{
				delay(200);
				lc.SetBias();
				delay(200);
				lc.GetLoads(load_cell_values);
			}																										//sets bias of load cell, zeroing it after platform is at zero point
			cout << "\nLoad Cell Zeroed\n";
			PrettyPrint::PrintLoad(load_cell_values);
		}

		mhp->MovePosition(VARIANT_TRUE, L"Retracted");
		while (mhp->Moving)
		{
		}
		cout << "\nMHP Retracted\n";

		//===== Set Velocity =====//
		MHPVelocity(mhp, lc);

		//===== Variables =====//
		int cycle = 1;																							//variable keeps track of which loop the MHP is on
		int cancel = 0;																							//variable to cancel moving loop if necessary
		string filestring;																						//user input path file name
		string savestring;																						//user input save file name
		string anglestring;																						//user input angles file name
		int loops = 0;																							//user input number of loops
		int load_loops = 20;																					//value that holds number of measurements are taken at each point
		cout << "\nWhat path would you like to follow?\n";
		cin >> filestring;
		string full_path = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/paths/NoseRibPaths/" + filestring + ".csv";	//full path name of where the path file should be
		cout << "\nWhat file name would you like to save to?\n";												//Save file name
		cin >> save_name;																						//Save file name input
		list<vector<double>> position;																			//declares a list of vectors to store all the position data
		vector<double> posvec_flip;																				//declares a vector that can store the value of a single position move
		vector<double> posvec_norm;																				//declares a vector that can store the value of a single position move
		int point_count = 1;
		vector<double> delXYZ = { 1, 1, 1 };

		ifstream pathfile(full_path);																		//path file that is used for below code
		if (pathfile.fail())
		{
			cout << "\nFile Name Does Not Exist\n\n";
			return;
		}
		cout << "================================================\n";
		cout << "Now on Loop: " << cycle << " of " << loops << "\n";
		string line;

		int i = 0;																							//variable to control looping through the 16 values of each line
		while (getline(pathfile, line) && cancel == 0)														//gets a line from the csv as a string ex: "1,0,0,0,0,1,0,0,0,0,1,0,10,0,0,1"
		{
			//============== Get Timestamp and store vector ==============//
			timestamp = duration_cast< milliseconds >(
				system_clock::now().time_since_epoch());
			float time = timestamp.count();
			all_timestamps.push_back(timestamp);

			stringstream ss(line);																			//
			while (ss.good() && cancel == 0)																//
			{
				string substr;																				//string variable to break apart the line of data, based on comma that separates each value
				getline(ss, substr, ',');																	//uses the line data gathered, and breaks apart each value
				double h = stod(substr);																	//converts string values to useable double values
				posvec_norm.push_back(h);																	//adds the value to the end of a vector
				i++;																						//increments a counting variable
				if (i == 16 && cancel == 0)
				{
					position.push_back(posvec_norm);														//adds the vector to the end of the list (once the vector has all 16 values)
					posvec_flip = MHPTranspose(posvec_norm);
					cout << "Moving to: " << posvec_flip[12] << ", " << posvec_flip[13] << ", " << posvec_flip[14] << "\n";
					cout << "Now on Point: " << point_count << "\n";
					variant_t v = VectorToVariant(posvec_flip);												//converts vector to useable variable to move platform
					mhp->MoveMatrix(VARIANT_TRUE, L"Reamer Nose", L"Reamer Nose Zero", v);							//uses v value to move the platform relative to the base by 1 mm in x

					//===== Checks how close current pose is to desired pose =====//
					variant_t current_pos = mhp->GetPositionMatrix(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");	//gets position of reamer with respect to reamer zero
					vector<double> current_pos_ = DoubleVariantToVector(current_pos);									//converts the get pos variant to vector, after the variant is converted from a array X [12], Y [13], Z [14]
					delXYZ[0] = posvec_flip[12] - current_pos_[12];
					delXYZ[1] = posvec_flip[13] - current_pos_[13];
					delXYZ[2] = posvec_flip[14] - current_pos_[14];
					
					while ( ((delXYZ[0] > 0.15 || delXYZ[0] < -0.15) ||
							(delXYZ[1] > 0.15 || delXYZ[1] < -0.15) ||
							(delXYZ[2] > 0.15 || delXYZ[2] < -0.15)) && cancel == 0)																	//could use "mhp->Moving" command
					{
						if (_kbhit())
						{
							int stop_char = _getch();
							if (stop_char == 27)															//esc button is pressed
							{
								mhp->Stop();
								cancel = 1;
								break;																		//breaks out of while look and cancels path
							}																				//this two if statements check if escape is pressed, if it is it will stop the robot, basically and estop
							if (stop_char == 32)															//if spacebar is hit
							{
								while (1)																	//goes into continuous while loop until space is pressed again
								{
									if (_kbhit())
									{
										int breakchar = _getch();
										if (stop_char == 32)
										{
											break;
										}
									}
								}
							}
						}																					//escape button press command

						lc.GetLoads(load_cell_values);														//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
						current_loads[0] = static_cast<double>(load_cell_values[0]);
						current_loads[1] = static_cast<double>(load_cell_values[1]);
						current_loads[2] = static_cast<double>(load_cell_values[2]);
						current_loads[3] = static_cast<double>(load_cell_values[3]);
						current_loads[4] = static_cast<double>(load_cell_values[4]);
						current_loads[5] = static_cast<double>(load_cell_values[5]);
						
						//===== Update Check difference of current to desired pose =====//
						variant_t current_pos = mhp->GetPositionMatrix(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");	//gets position of reamer with respect to reamer zero
						vector<double> current_pos_ = DoubleVariantToVector(current_pos);									//converts the get pos variant to vector, after the variant is converted from a array X [12], Y [13], Z [14]
						delXYZ[0] = posvec_flip[12] - current_pos_[12];
						delXYZ[1] = posvec_flip[13] - current_pos_[13];
						delXYZ[2] = posvec_flip[14] - current_pos_[14];
					}
					//delay(500);																			//delay used to allow the load cell and MHP to settle reduce inertial effects in (milliseconds) 500
					vector<double> current_pos_T = MHPTranspose(current_pos_);
					all_poses.push_back(current_pos_T);
					all_loads.push_back(current_loads);
					posvec_norm.clear();																	//clears the position vector so it can store the next line on the next loop
					MHPPrintCurrentPos(mhp, robot);														//function that prints a matrix
					i = 0;																					//counter
				}
			}

			point_count++;
		}

			//============ Store final timestamp =============//
			timestamp = duration_cast< milliseconds >(
				system_clock::now().time_since_epoch());
			float time = timestamp.count();
			all_timestamps.push_back(timestamp);
			timestamp = all_timestamps[1];


			//======= Get Final Last Position =======//
			variant_t current_pos = mhp->GetPositionMatrix(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");	//gets position of reamer with respect to reamer zero
			vector<double> current_pos_ = DoubleVariantToVector(current_pos);									//converts the get pos variant to vector, after the variant is converted from a array X [12], Y [13], Z [14]
			vector<double> current_pos_T = MHPTranspose(current_pos_);
			all_poses.push_back(current_pos_T);
			all_loads.push_back(current_loads);

		cout << "\n==============     Complete!     ==============\n\n";
		cout << "Do you want to retract robot to safe position 'y' or 'n'?\n\n";
		char retract;
		cin >> retract;
		if (retract == 'y')
		{
			mhp->MovePosition(VARIANT_TRUE, L"Retracted");
			while (mhp->Moving)
			{
			}
			cout << "\nMHP Retracted\n";
		}
	}
	catch (const _com_error& ex)
	{
		std::cout << "Error! Could Not Run Path!\nMake Sure MHP Is Turned On and Has Been Connected!\n\n";
	}

	int size = all_timestamps.size();

	//==== Saves results file of all current poses ====//
	string save_paths = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/results/NoseRibResults/" + save_name + "_results.csv";
	ofstream savefile(save_paths);																			//loops through each load file name to separate the loads
	string header = "Time (ms),r00,r01,r02,X (mm),r10,r11,r12,Y (mm),r20,r21,r22,Z (mm),Row 0,Row 1,Row 2,Row 3,\n";
	savefile << header;
	for (int row = 0; row < size; row++)																	//used to loop through all the vectors 
	{
		timestamp = all_timestamps[row] - all_timestamps[0];
		savefile << timestamp.count();
		savefile << ",";
		for (int column = 0; column <= 15; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile << all_poses[row][column];																//writes the specified element to the file
			savefile << ", ";																				//writes comma after each value in the file
		}
		savefile << "\n";																					//writes a new line in the file onces a vector is completed
	}

	//==== Saves loads file of all current poses ====//
	string save_loads = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/results/NoseRibLoads/" + save_name + "_loads.csv";
	ofstream savefile_loads(save_loads);																			//loops through each load file name to separate the loads
	string header_loads = "Time (ms),X (N),Y (N),Z (Y),Rx (Nm),Ry (Nm),Rz (Nm),\n";
	savefile_loads << header_loads;
	for (int row = 0; row < size; row++)																	//used to loop through all the vectors 
	{
		timestamp = all_timestamps[row] - all_timestamps[0];
		savefile_loads << timestamp.count();
		savefile_loads << ",";
		for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile_loads << all_loads[row][column];																//writes the specified element to the file
			savefile_loads << ", ";																				//writes comma after each value in the file
		}
		savefile_loads << "\n";																					//writes a new line in the file onces a vector is completed
	}
}
///////////////////////////////////////////////////////////////////////////////
////		       MHP Retract New Robot to Safe Low Position		       ////
///////////////////////////////////////////////////////////////////////////////
void MHPRetract(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
	MHPOpen(mhp,robot);
	mhp->MovePosition(VARIANT_TRUE, L"Retracted");
	cout << "\nMHP Retracted\n";
}

///////////////////////////////////////////////////////////////////////////////
////					           Test Code					           ////
///////////////////////////////////////////////////////////////////////////////
void TestCode(IHexapodPtr& mhp, Nano25E& lc, int robot)
{
	char mhp1 = 'n';
	cout << "\nWould you like to connect Robot Platform (y) or (n)?\n";
	cin >> mhp1;
	if (mhp1 == 'y')
	{
		MHPOpen(mhp, robot);
	}
	MHPZero(mhp, robot);																							//moves mhp to zero position
	float loads_msr[LOAD_CELL_DOF] = {};																	//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
	float loads_delta[LOAD_CELL_DOF] = {};																	//holds difference of desired and measured loads
	float loads_delta_converted[LOAD_CELL_DOF] = {};														//holds the delta loads after they have conversion factor applied
	char zero = 'n';
	while (mhp->Moving){};
	lc.GetLoads(loads_msr);
	delay(2000);
	PrettyPrint::PrintLoad(loads_msr);

	//======== Initilize Load Cell ========//
	cout << "\nWould you like to zero load cell (y) or (n)?\n";
	cin >> zero;
	if (zero == 'y')
	{
		cout << "\nDisconnect strips before zeroing load cell, press Any key and ENTER when ready\n";
		char t;
		cin >> t;
		lc.Stop();
		fprintf(stdout, "\nStarting load cell\n\n");
		lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
			Config::LOAD_CELL_TRANSFORMATION,
			Config::SAMPLE_RATE,
			Config::LOADCELL_CHANNEL);
		delay(2000);
		lc.SetBias();
		delay(2000);
		//initializing load cell code

		//=========== Zero Load Cell ==========//
		lc.GetLoads(loads_msr);
		while ((loads_msr[0] > 0.1 || loads_msr[0] < -0.1) ||
			(loads_msr[1] > 0.1 || loads_msr[1] < -0.1) ||
			(loads_msr[2] > 0.1 || loads_msr[2] < -0.1) ||
			(loads_msr[3] > 0.1 || loads_msr[3] < -0.1) ||
			(loads_msr[4] > 0.1 || loads_msr[4] < -0.1) ||
			(loads_msr[5] > 0.1 || loads_msr[5] < -0.1))
		{
			delay(200);
			lc.SetBias();
			delay(200);
			lc.GetLoads(loads_msr);
		}																										//sets bias of load cell, zeroing it after platform is at zero point
		cout << "\nLoad Cell Zeroed\n";
		delay(2000);
		PrettyPrint::PrintLoad(loads_msr);
	}
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;										//creates eigen matrix for current pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> final_pose_matrix;											//creates eigen matrix for result pose
	vector<vector<double>> lower_deadbands = MHPSetLowerDeadbands();										//creates vector of lower deadbands
	vector<vector<double>> upper_deadbands = MHPSetUpperDeadbands(mhp);										//creates vector of upper deadbands
	vector<vector<double>> limits = MHPSetLimits();															//creates vector of limit range, if inside these limits move on to next position
	vector<vector<double>> limits2 = MHPSetLimits2();															//creates vector of limit range, if inside these limits move on to next position
	vector<double> initial_pos_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };					//vector eventually used to as the starting position
	vector<double> next_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };						//creates vector for next pose
	string load_name;																						//variable to store file name that is to be loaded
	string save_name;																						//variable to store file name that is to be saved
	char hold = 'n';
	cout << "\nWhat load file do you wish to run?\n";
	cin >> load_name;
	cout << "\nWhat file name would you like to save to?\n";
	cin >> save_name;
	cout << "\nWould you like to hold position (y) or run path (n)?\n";										//variable to choose if you want to just hold a position (to see how robot reacts) or run path
	cin >> hold;
	if (hold != 'y' && hold != 'n')
	{
		hold = 'n';
	}
	string load_file = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/loads/" + load_name + ".csv";
	vector<vector<double>> loads_dsr;																		//vector that stores desired loads
	int load_loops = 1;																						//value that holds number of measurements are taken at each point
	vector<double> load_vec;																				//used to tranfer loads from a saved file being read to the loads_dsr vector of vectors
	vector<double> loads_sum = { 0, 0, 0, 0, 0, 0 };														//used to sum up a number of measured loads in quick succession
	vector<double> conversion_factor = MHPSetPControl();													//holds conversion factor for each load Fx,Fy,Fz,Mx,My,Mz
	int cancel = 0;
	variant_t pos = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");						//gets position of reamer with respect to reamer zero
	vector<double> pos_ = DoubleVariantToVector(pos);														//converts the get pos variant to vector, after the variant is converted from a array
	vector<double> current_pos_transpose = MHPTranspose(pos_);												//Transposes the matrix so it is in the conventional form
	vector<vector<double>> all_poses;																		//stores all values
	vector<double> current_and_dsr = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };			//holds single line of the current T matrix and the desired position (ie. X,Y,Z)
	vector<vector<double>> all_msr_loads;																	//holds all the measured loads
	vector<double> vec_loads_delta = { 0, 0, 0, 0, 0, 0 };
	vector<double> vec_loads_delta_converted = { 0, 0, 0, 0, 0, 0 };
	vector<double> vec_u_limits = { 0, 0, 0, 0, 0, 0 };
	vector<double> vec_l_limits = { 0, 0, 0, 0, 0, 0 };
	vector<vector<double>> all_loads_delta;
	vector<vector<double>> all_loads_delta_converted;
	vector<vector<double>> all_u_limits;
	vector<vector<double>> all_l_limits;
	vector<double> limits_boolean = { 0, 0, 0, 0, 0, 0 };													//vector to hold boolean results of limit check
	vector<double> limits_boolean2 = { 0, 0, 0, 0, 0, 0 };													//vector to hold boolean results of limit check
	double limits_boolean_sum = 0;																			//vector to hold limit checks of each load cell component '0' for false '1' for true
	double limits_boolean_sum2 = 0;																			//vector to hold limit checks of each load cell component '0' for false '1' for true
	double limit_control_value = 6;																			//value that hold the number of limits that need to be true in order for the target point to be resolved
	double limit_control_value2 = 6;																		//value that hold the number of limits that need to be true in order for the target point to be resolved

	//======== Integral/Derivative ========//
	milliseconds timestamp;																					//timestamp variable
	double time = 0;																						//hold current time value since January 1970
	vector<double> error = { 0, 0, 0, 0, 0, 0 };															//vector holds error for x,y,z,Rx,Ry,Rz
	vector<vector<double>> error_all;																		//sum all errors
	vector<milliseconds> all_timestamps;																	//holds vector of timestamps
	vector<double> time_sum;
	vector<double> integral = { 0, 0, 0, 0, 0, 0 };
	vector<double> derivative = { 0, 0, 0, 0, 0, 0 };
	
	//======= Dot Product Variables =======//
	vector<double> point1 = { 0, 0, 0 };																	//holds point i-2 in otherwords a points for two ago
	vector<double> point2 = { 0, 0, 0 };																	//holds point i-1 in otherwords the previous point
	vector<double> point3 = { 0, 0, 0 };																	//holds point i   in otherwords the current point
	int loopcount = 0;																						//keeps count of while loop counter
	vector<double> Vi = { 0, 0, 0 };																		//initial vector between point 2 and point 1
	vector<double> Vf = { 0, 0, 0 };																		//initial vector between point 3 and point 2
	int dot_switch_count = 0;																				//keeps count of number of times dot product switches direction
	double dot_product = 0;																					//holds next dot product value

	//========== Reads Load File ==========//
	ifstream loads(load_file);
	if (loads.fail())
	{
		cout << "File name does not exist!\n\n";															//If the file doesn't exist the loop stops
		return;
	}
	string line;																							//variable to get the a line from the csv file
	int i = 0;																								//variable to control looping through the 16 values of each line
	while (getline(loads, line))																			//gets a line from the csv as a string ex: "1,0,0,0,0,1,0,0,0,0,1,0,10,0,0,1"
	{
		stringstream ss(line);																				//
		while (ss.good())																					//
		{
			string substr;																					//string variable to break apart the line of data, based on comma that separates each value
			getline(ss, substr, ',');																		//uses the line data gathered, and breaks apart each value
			double h = stod(substr);																		//converts string values to useable double values
			load_vec.push_back(h);																			//adds the value to the end of a vector
			i++;																							//increments a counting variable
			if (i == 9)
			{
				loads_dsr.push_back(load_vec);																//adds the vector to the end of the list (once the vector has all 16 values)
				load_vec.clear();																			//clears the position vector so it can store the next line on the next loop
				i = 0;																						//counter
			}
		}
	}																										//Gets Loads from a csv file
	int length = loads_dsr.size();																			//gets the number of rows in dsr load file to know number of loops

											//===== Move To Starting Position =====//
													vector <double> initial_euler = { 0, 0, 0, 0, 0, 0 };
													cout << "\nEnter X(mm) intial offset\n";
													cin >> initial_euler[0];																					//enter intial X(mm) offset
													cout << "\nEnter Y(mm) intial offset\n";
													cin >> initial_euler[1];																					//enter intial Y(mm) offset
													cout << "\nEnter Z(mm) intial offset\n";
													cin >> initial_euler[2];																					//enter intial Z(mm) offset
													cout << "\nEnter Rx(deg) intial offset\n";
													cin >> initial_euler[3];																					//enter intial Rx(deg) offset
													cout << "\nEnter Ry(deg) intial offset\n";
													cin >> initial_euler[4];																					//enter intial Ry(deg) offset
													cout << "\nEnter Rz(deg) intial offset\n";
													cin >> initial_euler[5];																					//enter intial Rz(deg) offset

													//====== Create Load Cell matrix =======//
													Eigen::Matrix<float, 4, 4, Eigen::RowMajor> initial_matrix;
													float sA = static_cast<float>(sin(RAD(initial_euler[5])));						//Rz coverted to alpha angle
													float sB = static_cast<float>(sin(RAD(initial_euler[4])));						//Ry coverted to beta angle
													float sG = static_cast<float>(sin(RAD(initial_euler[3])));						//Rx coverted to gamma angle
													float cA = static_cast<float>(cos(RAD(initial_euler[5])));						//Rz coverted to alpha angle
													float cB = static_cast<float>(cos(RAD(initial_euler[4])));						//Ry coverted to beta angle
													float cG = static_cast<float>(cos(RAD(initial_euler[3])));						//Rx coverted to gamma angle
													initial_matrix <<
														cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), initial_euler[0],
														sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), initial_euler[1],
														-sB, cB*sG, cB*cG, initial_euler[2],
														0, 0, 0, 1;

													//======== Convert Transform Calculation into Variant ========//
													for (unsigned int i = 0; i < 16; ++i) {
														initial_pos_vec[i] = initial_matrix((i / 4), (i % 4));								//assign result pose back to vector
													}
													vector<double> initial_pos_vec_T = MHPTranspose(initial_pos_vec);							//transposes vector so can be sent to MHP using its conventions
													variant_t initial_pos_var = VectorToVariant(initial_pos_vec_T);							//converts vector to variant to be sent to MHP

													//======= Moves platform/stores & displays current pose ======//
													mhp->MoveMatrix(VARIANT_TRUE, L"Reamer", L"Reamer Zero", initial_pos_var);				//moves the platform based on the matrix sent to it in absolute terms

													while (mhp->Moving){};
													MHPPrintCurrentPos(mhp, robot);
													lc.GetLoads(loads_msr);																	//Gets initial loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_initial
													lc.GetLoads(loads_delta);																//Initializes the delta loads variable, over-written later
													cout << "\n3..."; delay(1000); 
													cout << "\n2..."; delay(1000); 
													cout << "\n1..."; delay(1000);

													
	//======== Main Algorithm Loop ========//
	for (int row = 0; row < length; row++)
	{
		//===== Move to Zero/Zero Load Cell =====//
		cout << "\n=======================================================\n";
		cout << "\nNow on point: " << row + 1 << " of " << length << "\n";
		while (1)
		{
			loopcount++;

			//========== Get Initial Loads ==========//
			for (int load_count = 1; load_count <= load_loops; load_count++)							//takes the average of a number of loops
			{
				lc.GetLoads(loads_msr);																	//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
				loads_sum[0] = static_cast<double>(loads_msr[0]) + loads_sum[0];
				loads_sum[1] = static_cast<double>(loads_msr[1]) + loads_sum[1];
				loads_sum[2] = static_cast<double>(loads_msr[2]) + loads_sum[2];
				loads_sum[3] = static_cast<double>(loads_msr[3]) + loads_sum[3];
				loads_sum[4] = static_cast<double>(loads_msr[4]) + loads_sum[4];
				loads_sum[5] = static_cast<double>(loads_msr[5]) + loads_sum[5];						//converts the float variables to useable vector double and sums up each individual load
				//delay(2);
			}
			for (int z = 0; z <= 5; z++)
			{
				loads_sum[z] = loads_sum[z] / load_loops;												//averages each individual load
			}
			all_msr_loads.push_back(loads_sum);															//adds vector of the measured loads to the end of this variable
			all_msr_loads.push_back(loads_dsr[row]);													//adds vector of the desired loads to the end of this variable 

			//====== Dsr - Msr Load Difference ======//
			for (int term = 0; term < 6; term++)
			{
				loads_delta[term] = loads_sum[term] - loads_dsr[row][term];								//measured minus desired, since loads are difference is opposite to direction of travel
				error[term] = loads_delta[term];
			}
			error_all.push_back(error);
			loads_sum = { 0, 0, 0, 0, 0, 0 };															//resets the load sum so that previous load values do not skew values

			//============== Get Timestamp and store vector ==============//
			timestamp = duration_cast< milliseconds >(
				system_clock::now().time_since_epoch());
				all_timestamps.push_back(timestamp);
			float time = timestamp.count();																			//to make useable variable
			if (loopcount == 1)
			{
				time = 0;
			}
			else 
			{
				timestamp = all_timestamps[loopcount-1] - all_timestamps[loopcount-2];					//loopcount is one step ahead need to subtract 1 and 2 respectively
				time = timestamp.count();
			}
			time_sum.push_back(time);

			MHP_ID_Controller(time_sum, error_all, integral, derivative);																//call function for ID controller
			
			//=========== Load Corrections ==========//
			for (int term = 0; term < 6; term++)
			{
				loads_delta_converted[term] = loads_delta[term] * conversion_factor[term];
			}

			//pose:																							//label used in the catch function to essential retry a failed step
			try																							//try and catch loop needed in order to prevent the program from stopping if a position can't be reached
			{
				//======== Check Upper Deadband =========//
				upper_deadbands = MHPSetUpperDeadbands(mhp);
				for (int term = 0; term < 6; term++)
				{
					if (loads_delta_converted[term] < 0
						&& loads_delta_converted[term] < upper_deadbands[term][0] * 0.05)										//checks if value is negative and > max neg deadband creating a threshold limit
					{
						loads_delta_converted[term] = upper_deadbands[term][0] * 0.05;											//sets value to a max step size, limits correction
					}
					if (loads_delta_converted[term] > 0
						&& loads_delta_converted[term] > upper_deadbands[term][1] * 0.05)										//checks if value is negative and > max neg deadband creating a threshold limit
					{
						loads_delta_converted[term] = upper_deadbands[term][1] * 0.05;											//sets value to a max step size, limits correction
					}
				}

				for (int term = 0; term < 6; term++)
				{
					vec_loads_delta[term] = static_cast<double>(loads_delta[term]);
					vec_loads_delta_converted[term] = static_cast<double>(loads_delta_converted[term]);
					vec_u_limits[term] = static_cast<double>(upper_deadbands[term][1]);
					vec_l_limits[term] = static_cast<double>(upper_deadbands[term][0]);
				}
				all_loads_delta.push_back(vec_loads_delta);
				all_loads_delta_converted.push_back(vec_loads_delta_converted);
				all_u_limits.push_back(vec_u_limits);
				all_l_limits.push_back(vec_l_limits);

				//========= Gets Current Position From MHP Converts it =========//
				variant_t pos_var = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");		//gets a variant that is 2D of current position
				vector<double> cur_pos = DoubleVariantToVector(pos_var);										//currents 2D variant to a useable 1D 16 term vector
				cur_pos = MHPTranspose(cur_pos);																//transposes matrix vector to standard matrix form
				Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;
				current_pose_matrix <<
					cur_pos[0], cur_pos[1], cur_pos[2], cur_pos[3],    // [3] -> x
					cur_pos[4], cur_pos[5], cur_pos[6], cur_pos[7],    // [7] -> y
					cur_pos[8], cur_pos[9], cur_pos[10], cur_pos[11],  //[11] -> z
					cur_pos[12], cur_pos[13], cur_pos[14], cur_pos[15];									//turns vector into matrix for easy use of matrix multiplication function

				//====== Create Load Cell matrix =======//
				Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
				float sA = static_cast<float>(sin(RAD(loads_delta_converted[5])));						//Rz coverted to alpha angle
				float sB = static_cast<float>(sin(RAD(loads_delta_converted[4])));						//Ry coverted to beta angle
				float sG = static_cast<float>(sin(RAD(loads_delta_converted[3])));						//Rx coverted to gamma angle
				float cA = static_cast<float>(cos(RAD(loads_delta_converted[5])));						//Rz coverted to alpha angle
				float cB = static_cast<float>(cos(RAD(loads_delta_converted[4])));						//Ry coverted to beta angle
				float cG = static_cast<float>(cos(RAD(loads_delta_converted[3])));						//Rx coverted to gamma angle
				lc_matrix <<
					cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), loads_delta_converted[0],
					sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), loads_delta_converted[1],
					-sB, cB*sG, cB*cG, loads_delta_converted[2],
					0, 0, 0, 1;																			//load cell T matrix for small corrections

				//====== Matrix Multiplication To Base Coordinate System =====//
				final_pose_matrix = current_pose_matrix * lc_matrix;									//result pose calculation, mutliplcation of current and load cell T matrix

				//======== Convert Transform Calculation into Variant ========//
				for (unsigned int i = 0; i < 16; ++i) {
					next_pose_vec[i] = final_pose_matrix((i / 4), (i % 4));								//assign result pose back to vector
				}
				vector<double> next_pose_vec_trans = MHPTranspose(next_pose_vec);						//transposes vector so can be sent to MHP using its conventions
				variant_t next_pose_var = VectorToVariant(next_pose_vec_trans);							//converts vector to variant to be sent to MHP

				//======= Moves platform/stores & displays current pose ======//
				mhp->MoveMatrix(VARIANT_TRUE, L"Reamer", L"Reamer Zero", next_pose_var);				//moves the platform based on the matrix sent to it in absolute terms

				//=============== Dot Product ===============//
				point1 = point2;
				point2 = point3;
				point3 = { cur_pos[3], cur_pos[7], cur_pos[11] };
				if (loopcount > 2 && hold == 'n')
				{
					Vi = { point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2] };
					Vf = { point3[0] - point2[0], point3[1] - point2[1], point3[2] - point2[2] };
					dot_product = (Vf[0] * Vi[0]) + (Vf[1] * Vi[1]) + (Vf[2] * Vi[2]);
					if (dot_product < 0)
					{
						dot_switch_count++;
					}
					//cout << "\n" << dot_switch_count << "\n";
					if (dot_switch_count >= 5 && mhp->Moving)
					{
						break;
					}
				}

				//=========== Keyboard Hits ==========//
				if (_kbhit())
				{
					int stop_char = _getch();
					if (stop_char == 27)
					{
						mhp->Stop();
						goto LABEL;																		//exits program with ESC button press
					}
					if (stop_char == 32)
					{
						break;																			//skips to next position with space bar press
					}
				}																						//escape button press command
				//MHPPrintCurrentPos(mhp);																//prints current pose
				pos = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");				//gets position of platform with respect to platform zero
				pos_ = DoubleVariantToVector(pos);														//converts the get pos variant to vector, after the variant is converted from a array
				current_pos_transpose = MHPTranspose(pos_);												//Transposes the matrix so it is in the conventional form
				for (int r = 0; r <= 15; r++)
				{
					current_and_dsr[r] = current_pos_transpose[r];
				}
				current_and_dsr[16] = loads_dsr[row][6];
				current_and_dsr[17] = loads_dsr[row][7];
				current_and_dsr[18] = loads_dsr[row][8];
				all_poses.push_back(current_and_dsr);

				if ((loads_delta[0] < limits2[0][1] && loads_delta[0] > limits2[0][0]) &&
					(loads_delta[1] < limits2[1][1] && loads_delta[1] > limits2[1][0]) &&
					(loads_delta[2] < limits2[2][1] && loads_delta[2] > limits2[2][0]) &&
					(loads_delta[3] < limits2[3][1] && loads_delta[3] > limits2[3][0]) &&
					(loads_delta[4] < limits2[4][1] && loads_delta[4] > limits2[4][0]) &&
					(loads_delta[5] < limits2[5][1] && loads_delta[5] > limits2[5][0]) &&
					hold == 'n')
				{
					break;
				}
			}
			catch (const _com_error& ex)
			{
				cout << "**************************  !!! OUT OF RANGE !!!  **************************\n\n";
				for (int term = 0; term < 6; term++)
				{
					loads_delta_converted[term] = loads_delta_converted[term] * 0.05;
				}
				if (_kbhit())
				{
					int stop_char = _getch();
					if (stop_char == 27)
					{
						mhp->Stop();
						goto LABEL;																		//exits program with ESC button press
					}
					if (stop_char == 32)
					{
						break;																			//skips to next position with space bar press
					}
				}
			}
		}
		loopcount = 0;
		dot_switch_count = 0;
	}
	MHPPrintCurrentPos(mhp, robot);
	//======= End of the Loop, Writing Results =======//
LABEL:
	mhp->Stop();
	//======= Store and display final position =======//
	cout << "\nFinal ";
	MHPPrintCurrentPos(mhp, robot);
	cout << "Complete!\n\n";
	pos = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");								//gets position of platform with respect to platform zero
	pos_ = DoubleVariantToVector(pos);																		//converts the get pos variant to vector, after the variant is converted from a array
	current_pos_transpose = MHPTranspose(pos_);																//Transposes the matrix so it is in the conventional form
	for (int r = 0; r <= 15; r++)
	{
		current_and_dsr[r] = current_pos_transpose[r];
	}
	current_and_dsr[16] = loads_dsr[length - 1][6];
	current_and_dsr[17] = loads_dsr[length - 1][7];
	current_and_dsr[18] = loads_dsr[length - 1][8];
	all_poses.push_back(current_and_dsr);

	//===== Get and store final load measurement =====//
	for (int load_count = 1; load_count <= load_loops; load_count++)
	{
		lc.GetLoads(loads_msr);																				//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
		loads_sum[0] = static_cast<double>(loads_msr[0]) + loads_sum[0];
		loads_sum[1] = static_cast<double>(loads_msr[1]) + loads_sum[1];
		loads_sum[2] = static_cast<double>(loads_msr[2]) + loads_sum[2];
		loads_sum[3] = static_cast<double>(loads_msr[3]) + loads_sum[3];
		loads_sum[4] = static_cast<double>(loads_msr[4]) + loads_sum[4];
		loads_sum[5] = static_cast<double>(loads_msr[5]) + loads_sum[5];
		delay(2);
	}
	for (int z = 0; z <= 5; z++)
	{
		loads_sum[z] = loads_sum[z] / load_loops;															//converts the float variables to useable vector double
	}
	all_msr_loads.push_back(loads_sum);
	all_msr_loads.push_back(loads_dsr[length - 1]);

	//============ Store final timestamp =============//
	timestamp = duration_cast< milliseconds >(
		system_clock::now().time_since_epoch());
		time = timestamp.count();																			//to make useable variable
	all_timestamps.push_back(timestamp);
	int size = all_timestamps.size();

	//==== Saves results file of all current poses ====//
	string save_paths = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/results/" + save_name + "_results.csv";
	ofstream savefile(save_paths);																			//loops through each load file name to separate the loads
	string header_int = " , ,Intial Positions\nX (mm),Y (mm),Z (mm),Rx (deg), Ry (deg), Rz (deg)\n";
	string header = "Time (ms),r00,r01,r02,X (mm),r10,r11,r12,Y (mm),r20,r21,r22,Z (mm),Row 0,Row 1,Row 2,Row 3,"
		"Dsr X (mm),Dsr Y (mm),Dsr Z (mm),Msr X (N),Dsr X (N),Msr Y (N),Dsr Y (N),Msr Z (N),Dsr Z (N),"
		"Msr Rx (Nm),Dsr Rx (Nm),Msr Ry (Nm),Dsr Ry (Nm),Msr Rz (Nm),Dsr Rz (Nm)\n";
	savefile << header_int;
	savefile << initial_euler[0]; savefile << ", "; savefile << initial_euler[1]; savefile << ", ";
	savefile << initial_euler[2]; savefile << ", "; savefile << initial_euler[3]; savefile << ", ";
	savefile << initial_euler[4]; savefile << ", "; savefile << initial_euler[5]; savefile << ", \n";
	savefile << header;

	for (int row = 0; row < size; row++)																	//used to loop through all the vectors 
	{
		timestamp = all_timestamps[row] - all_timestamps[0];
		savefile << timestamp.count();
		savefile << ",";
		for (int column = 0; column <= 18; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile << all_poses[row][column];																//writes the specified element to the file
			savefile << ", ";																				//writes comma after each value in the file
		}
		for (int column = 0; column <= 5; column++)
		{
			savefile << all_msr_loads[row * 2][column];														//writes the specified element to the file
			savefile << ", ";																				//writes comma after each value in the file
			savefile << all_msr_loads[(row * 2) + 1][column];
			savefile << ", ";
		}
		savefile << "\n";																					//writes a new line in the file onces a vector is completed
	};

	//==== Save Limits and load differences ====//
	string save_paths1 = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/results/" + save_name + "_results2.csv";
	ofstream savefile1(save_paths1);																				//loops through each load file name to separate the loads
	string header1 = "Delta X,Delta Y,Delta Z,Delta Rx,Delta Ry,Delta Rz,ConDel X,ConDel Y,ConDel Z,ConDel Rx,ConDel Ry,ConDel Rz,"
		"Lower X,Upper X,Lower Y,Upper Y,Lower Z,Upper Z,Lower Rx,Upper Rx,Lower Ry,Upper Ry,Lower Rz,Upper Rz\n";
	savefile1 << header1;
	int size1 = all_loads_delta.size();
	for (int row = 0; row < size1 - 1; row++)																		//used to loop through all the vectors 
	{
		for (int column = 0; column <= 5; column++)																	//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile1 << all_loads_delta[row][column];																//writes the specified element to the file
			savefile1 << ", ";																						//writes comma after each value in the file
		}
		for (int column = 0; column <= 5; column++)
		{
			savefile1 << all_loads_delta_converted[row][column];													//writes the specified element to the file
			savefile1 << ", ";																						//writes comma after each value in the file
		}
		for (int column = 0; column <= 5; column++)
		{
			savefile1 << all_l_limits[row][column];
			savefile1 << ", ";
			savefile1 << all_u_limits[row][column];
			savefile1 << ", ";
		}
		savefile1 << "\n";																					//writes a new line in the file onces a vector is completed
	};
}


//////////////////////////////////////////////////////////////////////////////////////
////                             Converstion Functions                            ////
//////////////////////////////////////////////////////////////////////////////////////
//=================== Vector to Variant ====================//
_variant_t VectorToVariant(vector<double>& vec)
{
	size_t count = vec.size();
	CComSafeArray<double> sa((ULONG)count);
	for (size_t i = 0; i < count; i++)
	{
		sa.SetAt((LONG)i, vec[i]);
	}
	_variant_t var;
	CComVariant(sa).Detach(&var);
	return var;
}
//=================== Varient to Vector ====================//
vector<double> VariantToVector(const _variant_t& var)
{
	CComSafeArray<double> SafeArray;
	SafeArray.Attach(var.parray);
	size_t count = SafeArray.GetCount();
	std::vector<double> vec(count);
	for (size_t i = 0; i < count; i++)
	{
		vec[i] = SafeArray.GetAt((ULONG)i);
	}
	SafeArray.Detach();
	return vec;
}
//=================== T-Flip to T-Normal ===================//
vector<double> TFlipToTNorm(vector<double>& flip)
{
	//  T-matrix Flip		        T-matrix Normal			Note: matrix arranged as single vector left to right row then top to bottom column
	//  r1, r2, r3, 0				 r1, r2, r3, X									flip[0] = r1,			norm[0] = r1;     {  0,  1,  2,  3,
	//  r4, r5, r6, 0	  ---->		 r4, r5, r6, Y									flip[1] = r2,			norm[1] = r2;		 4,  5,  6,  7,
	//  r7, r8, r9, 0				 r7, r8, r9, Z									flip[2] = r3,			norm[2] = r3;		 8,  9, 10, 11,
	//   X,  Y,  Z, 1				  0,  0,  0, 1									flip[3] =  0,			norm[3] =  X;		12, 13, 14, 15, }
	//																				flip[4] = r4,			norm[4] = r4;
	//																					  ⋮						  ⋮

	vector<double> norm = flip;
	norm[3] = flip[12];
	norm[7] = flip[13];
	norm[11] = flip[14];
	norm[12] = 0;
	norm[13] = 0;
	norm[14] = 0;
	return norm;
}
//=================== T-Normal to T-Flip ===================//
vector<double> TNormToTFlip(vector<double>& norm)
{
	//  T-matrix Normal		        T-matrix Flip			Note: matrix arranged as single vector left to right row then top to bottom column
	//  r1, r2, r3, X				 r1, r2, r3, 0									flip[0] = r1,			norm[0] = r1;
	//  r4, r5, r6, Y	  ---->		 r4, r5, r6, 0									flip[1] = r2,			norm[1] = r2;
	//  r7, r8, r9, Z				 r7, r8, r9, 0									flip[2] = r3,			norm[2] = r3;
	//   0,  0,  0, 1				  X,  Y,  Z, 1									flip[3] =  0,			norm[3] =  X;
	//																				flip[4] = r4,			norm[4] = r4;
	//																					  ⋮						  ⋮

	vector<double> flip = norm;
	flip[12] = norm[3];
	flip[13] = norm[7];
	flip[14] = norm[11];
	flip[3] = 0;
	flip[7] = 0;
	flip[11] = 0;
	return flip;
}
//================ Double Varient to Vector ================//
vector<double> DoubleVariantToVector(const _variant_t& var)
{
	CComSafeArray<double> SafeArray;
	SafeArray.Attach(var.parray);
	double cElement;
	LONG aIndex[2];
	int count = 0;
	std::vector<double> vec(16);
	for (int x = 0; x < 4; x++)
	{
		for (int y = 0; y < 4; y++)
		{
			aIndex[0] = x;
			aIndex[1] = y;
			SafeArray.MultiDimGetAt(aIndex, cElement); //HRESULT hr = 
			//ATLASSERT(hr == S_OK);
			//ATLASSERT(cElement == vec[count]);
			vec[count] = cElement;
			count++;
		}
	}
	SafeArray.Detach();
	return vec;
}
//================ Right To Left Coordinates ===============//
_variant_t RightToLeft(const _variant_t& var, float loads[6])
{
	float sA = static_cast<float>(sin(RAD(loads[5])));								//Rz coverted to alpha angle
	float sB = static_cast<float>(sin(RAD(loads[4])));								//Ry coverted to beta angle
	float sG = static_cast<float>(sin(RAD(loads[3])));								//Rx coverted to gamma angle
	float cA = static_cast<float>(cos(RAD(loads[5])));								//Rz coverted to alpha angle
	float cB = static_cast<float>(cos(RAD(loads[4])));								//Ry coverted to beta angle
	float cG = static_cast<float>(cos(RAD(loads[3])));
	vector<double> vec = VariantToVector(var);
	if ((cA*sB*sG) - (sA*cG) == 0)
	{
		vec[1] = 0;
	}
	else
	{
		vec[1] = vec[1] * ((cA*sB*sG) + (sA*cG)) / ((cA*sB*sG) - (sA*cG));
	}
	if ((cA*sB*cG) + (sA*sG) == 0)
	{
		vec[2] = 0;
	}
	else
	{
		vec[2] = vec[2] * ((sA*sG) - (cA*sB*cG)) / ((cA*sB*cG) + (sA*sG));
	}
	vec[4] *= -1;
	if ((sA*sB*sG) + (cA*cG) == 0)
	{
		vec[5] = 0;
	}
	else
	{
		vec[5] = vec[5] * ((cA*cG) - (sA*sB*sG)) / ((sA*sB*sG) + (cA*cG));
	}
	if ((sA*sB*cG) - (cA*sG) == 0)
	{
		vec[6] = 0;
	}
	else
	{
		vec[6] = vec[6] * ((cA*sG) + (sA*sB*cG)) / ((sA*sB*cG) - (cA*sG));
	}
	vec[8] *= -1;
	vec[9] *= -1;
	variant_t var_return = VectorToVariant(vec);
	return var_return;
}
//================ Left To Right Coordinates ===============//
_variant_t LeftToRight(const _variant_t& var, float loads[6])
{
	float sA = static_cast<float>(sin(RAD(loads[5])));								//Rz coverted to alpha angle
	float sB = static_cast<float>(sin(RAD(loads[4])));								//Ry coverted to beta angle
	float sG = static_cast<float>(sin(RAD(loads[3])));								//Rx coverted to gamma angle
	float cA = static_cast<float>(cos(RAD(loads[5])));								//Rz coverted to alpha angle
	float cB = static_cast<float>(cos(RAD(loads[4])));								//Ry coverted to beta angle
	float cG = static_cast<float>(cos(RAD(loads[3])));
	vector<double> vec = VariantToVector(var);
	if ((cA*sB*sG) + (sA*cG) == 0)
	{
		vec[1] = 0;
	}
	else
	{
		vec[1] = vec[1] * ((cA*sB*sG) - (sA*cG)) / ((cA*sB*sG) + (sA*cG));
	}
	if ((sA*sG) - (cA*sB*cG) == 0)
	{
		vec[2] = 0;
	}
	else
	{
		vec[2] = vec[2] * ((cA*sB*cG) + (sA*sG)) / ((sA*sG) - (cA*sB*cG));
	}
	vec[4] = vec[4] * -1;
	if ((cA*cG) - (sA*sB*sG) == 0)
	{
		vec[5] = 0;
	}
	else
	{
		vec[5] = vec[5] * ((sA*sB*sG) + (cA*cG)) / ((cA*cG) - (sA*sB*sG));
	}
	if ((cA*sG) + (sA*sB*cG) == 0)
	{
		vec[6] = 0;
	}
	else
	{
		vec[6] = vec[6] * ((sA*sB*cG) - (cA*sG)) / ((cA*sG) + (sA*sB*cG));
	}
	vec[8] = vec[8] * -1;
	vec[9] = vec[9] * -1;
	variant_t var_return = VectorToVariant(vec);
	return var_return;
}
//================= Print Current Position =================//
void MHPPrintCurrentPos(IHexapodPtr& mhp, int robot)
{
	variant_t pos = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets position of reamer with respect to reamer zero
	vector<double> pos_ = DoubleVariantToVector(pos);										//converts the get pos variant to vector, after the variant is converted from a array
	vector<double> current_pos_transpose = MHPTranspose(pos_);								//Transposes the matrix so it is in the conventional form
	cout << "Current Position: \n";
	for (int i = 0; i < 16; i++)
	{
		std::cout << std::setprecision(4)<< std::fixed;										//sets number of decimal points of numbers being displayed
		if (current_pos_transpose[i] >= 0)
		{
			cout << " ";																	//adds a space if number doesn't have negative sign in order to keep everything in line
		}
		cout << current_pos_transpose[i] << ", ";
		if (i == 3 || i == 7 || i == 11 || i == 15)											//after 4 terms prints to line to get the 4x4 matrix grid
		{
			cout << "\n";
		}
	}
	cout << "\n";
	std::cout << std::setprecision(4) << std::defaultfloat;									//sets number of decimal points of numbers being displayed
}
//================ MHP Set Lower Deadbands =================//
vector<vector<double>> MHPSetLowerDeadbands()																				//used for functions that need wider limits to not have jittering													
{
	vector<vector<double>> deadbands =
		//	  {x_l, x_h}  {y_l, y_h}  {z_l, z_h}  {rx_l, rx_h}   {ry_l, ry_h}    {rz_l, rz_h}								// l=lower limit, h=higher limit //limits around zero used to know when a point has reached its target
	{ { -0.06, 0.06 }, { -0.06, 0.06 }, { -0.06, 0.06 }, { -0.02, 0.02 }, { -0.02, 0.02 }, { -0.02, 0.02 } };				//deadbands 	
	return deadbands;
}
//================ MHP Set Upper Deadbands =================//
vector<vector<double>> MHPSetUpperDeadbands(IHexapodPtr& mhp)
{
	variant_t pose = mhp->GetPositionTuple(PositionActual, L"Reamer Nose", L"Reamer Nose Zero");
	variant_t poslim;
	variant_t neglim;
	mhp->GetMaxMoveDistanceTuple(PositionActual, L"Reamer Nose", L"Reamer Nose Zero", pose, &neglim, &poslim);
	vector<double> poslim_vec = VariantToVector(poslim);
	vector<double> neglim_vec = VariantToVector(neglim);
	poslim_vec[3] = poslim_vec[3] * 180 / PI;
	poslim_vec[4] = poslim_vec[4] * 180 / PI;
	poslim_vec[5] = poslim_vec[5] * 180 / PI;
	neglim_vec[3] = neglim_vec[3] * 180 / PI;
	neglim_vec[4] = neglim_vec[4] * 180 / PI;
	neglim_vec[5] = neglim_vec[5] * 180 / PI;

	vector<vector<double>> deadbands =
		//	  {x_l, x_h}  {y_l, y_h}  {z_l, z_h}  {rx_l, rx_h}   {ry_l, ry_h}    {rz_l, rz_h}		// l=lower limit (negtive), h=higher limit (positive)
	{ { neglim_vec[0], poslim_vec[0] }, { neglim_vec[1], poslim_vec[1] }, { neglim_vec[2], poslim_vec[2] }, { neglim_vec[3], poslim_vec[3] }, { neglim_vec[4], poslim_vec[4] }, { neglim_vec[5], poslim_vec[5] } };	//deadbands
	return deadbands;
}
//============ MHP Set Limits Load Difference ==============//
vector<vector<double>> MHPSetLimits()
{
	vector<vector<double>> limits =
		//	  {x_l, x_h}  {y_l, y_h}  {z_l, z_h}  {rx_l, rx_h}   {ry_l, ry_h}    {rz_l, rz_h}								// l=lower limit, h=higher limit 
	{ { -0.2, 0.2 }, { -0.2, 0.2 }, { -0.35, 0.35 }, { -0.05, 0.05 }, { -0.035, 0.035 }, { -0.05, 0.05 } };			//deadbands
	return limits;
}
//============ MHP Set Limits Load Difference ==============//
vector<vector<double>> MHPSetLimits2()
{
	vector<vector<double>> limits =
		//	  {x_l, x_h}  {y_l, y_h}  {z_l, z_h}  {rx_l, rx_h}   {ry_l, ry_h}    {rz_l, rz_h}		// l=lower limit, h=higher limit 
	{ { -0.15, 0.15 }, { -0.15, 0.15 }, { -0.1, 0.1 }, { -0.007, 0.007 }, { -0.007, 0.007 }, { -0.007, 0.007 } };				//deadbands { { -0.1, 0.1 }, { -0.1, 0.1 }, { -0.05, 0.05 }, { -0.007, 0.007 }, { -0.007, 0.007 }, { -0.007, 0.007 } };	
	return limits;
}
//================= MHP Sets the P Control =================//
vector<double> MHPSetPControl()
{
							  //Fx, Fy, Fz, Rx, Ry, Rz
	vector<double> p_control = { 2, 2, 1, 0.1, 0.1, 0.1};				//0.015, 0.025, 0.05	//4,4,4,5,5,5		//proporital controller vector
	return p_control;
}
//===================== MHP Nerd Stats =====================//
void MHPNerdStats(vector<vector<vector<double>>>& complete_loads, int& length, int& loops, string& save_file)
{
	string header = "Name: Stats For Nerds";
	vector<double> sum_vec;
	double st_dev = 0;
	vector<double> st_devs;
	vector<vector<double>> all_st_dev;
	double mean = 0;
	vector<double> means;
	vector<vector<double>> all_means;
	int run = 0;
	string save_paths = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/loads/" + save_file + "_NerdStats.csv";
	//====== Mean Calculation ======//
	for (int row = 0; row < length; row++)
	{
		for (int column = 0; column <= 5; column++)
		{
			for (run = 0; run < loops; run++)
			{
				mean += complete_loads[run][row][column];
			}
			mean = mean / loops;
			means.push_back(mean);
			mean = 0;
		}
		all_means.push_back(means);
		means.clear();
	}
	//====== St Dev Calculation ======//
	for (int row = 0; row < length; row++)
	{
		for (int column = 0; column <= 5; column++)
		{
			for (run = 0; run < loops; run++)
			{
				st_dev = complete_loads[run][row][column] - all_means[row][column];
				st_dev = st_dev*st_dev;
				sum_vec.push_back(st_dev);																//vector of (x-mean)^2
			}
			st_dev = 0;
			for (run = 0; run < loops; run++)
			{
				st_dev += sum_vec[run];																	//sum up (x-mean)^2
			}
			sum_vec.clear();
			st_dev = st_dev / (loops);																//divides the summation by number of points
			st_dev = sqrt(st_dev);
			st_devs.push_back(st_dev);
			st_dev = 0;
		}
		all_st_dev.push_back(st_devs);
		st_devs.clear();
	}

	//====== writing to file ======//
	ofstream savefile(save_paths);																		//loops through each load file name to separate the loads
	savefile << header;
	savefile << "\nNumber of Loops Run: ";
	savefile << loops;
	savefile << "\nStat: Mean\n";
	savefile << "Point,	Fx,		Fy,			Fz,		Tx,			Ty,			Tz,\n";
	for (int row = 0; row < length; row++)																//used to loop through all the vectors 
	{
		if (row < 10){savefile << " ";}
		savefile << row;
		savefile << ",	";
		for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile << all_means[row][column];															//writes the specified element to the file
			if (column != 5)
			{
				savefile << setprecision(6)<<",    	";													//writes comma after each value in the file
			}
		}
		savefile << "\n";																				//writes a new line in the file onces a vector is completed
	}
	savefile << "\n\n";																					//writes a new line in the file onces a vector is completed
	savefile << "Stat: Standard Deviation\n";
	savefile << "Point,		Fx,		Fy,			Fz,			Tx,			Ty,			Tz,\n";
	for (int row = 0; row < length; row++)																//used to loop through all the vectors 
	{
		if (row < 10){ savefile << " "; }
		savefile << row;
		savefile << ",	";
		for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile << all_st_dev[row][column];														//writes the specified element to the file
			if (column != 5)
			{
				savefile << setprecision(6)<<",    	";																		//writes comma after each value in the file
			}
		}
		savefile << "\n";																				//writes a new line in the file onces a vector is completed
	}
}
//===================== MHP Transpose ======================//
vector<double> MHPTranspose(vector<double>& matrix)
{
	vector<double> matrix_transpose = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	//		Matrix						  Transpose
	//  0,  1,  2,  3,					0,  4,  8, 12,
	//  4,  5,  6,  7,		--->		1,  5,  9, 13,
	//  8,  9, 10, 11,					2,  6, 10, 14,
	// 12, 13, 14, 15,				    3,  7, 11, 15,
	matrix_transpose[0] = matrix[0];
	matrix_transpose[1] = matrix[4];
	matrix_transpose[2] = matrix[8];
	matrix_transpose[3] = matrix[12];
	matrix_transpose[4] = matrix[1];
	matrix_transpose[5] = matrix[5];
	matrix_transpose[6] = matrix[9];
	matrix_transpose[7] = matrix[13];
	matrix_transpose[8] = matrix[2];
	matrix_transpose[9] = matrix[6];
	matrix_transpose[10] = matrix[10];
	matrix_transpose[11] = matrix[14];
	matrix_transpose[12] = matrix[3];
	matrix_transpose[13] = matrix[7];
	matrix_transpose[14] = matrix[11];
	matrix_transpose[15] = matrix[15];

	return matrix_transpose;
}
//====== MHP Digitize and Coordinate System Transform ======//
vector<double> MHPDigitize(IHexapodPtr& mhp, Nano25E& lc)
{
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> final_pose_matrix;									//creates eigen matrix for result pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> current_pose_matrix;								//creates eigen matrix for current pose
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> platform_zero_matrix;								//creates eigen matrix for platform zero

	float load_cell_values[LOAD_CELL_DOF] = {};														//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
	float load_cell_delta[LOAD_CELL_DOF] = {};
	float load_cell_zero[LOAD_CELL_DOF] = {};
	float load_cell_current[LOAD_CELL_DOF] = {};
	float load_cell_pid[LOAD_CELL_DOF] = {};
	vector<double> next_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };				//creates vector for next pose
	vector<double> current_pose_vec = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };			//creates vector for next pose
	variant_t CurrentPos_var = VectorToVariant(current_pose_vec);
	vector<double> load_vec = { 0, 0, 0, 0, 0, 0 };
	vector<vector<double>> loads1;
	vector<vector<double>> position;

	//======== P Controls ========//
	//p_x,  p_y,  p_z, p_rx, p_ry, p_rz
	vector<double> p_control = MHPSetPControl();													//proporital controller vector

	//====== Deadbands ======//
	vector<vector<double>> deadbands = { { -10, 10 }, { -14, 14 }, { -8, 8 }, { -4, 4 }, { -4, 4 }, { -2, 2 }, };	//set deadbands

	//==== File Name ====//
	string file_name;
	cout << "\nWhat would you like to name the save file as?\n";
	cin >> file_name;

	float loads_msr[LOAD_CELL_DOF] = {};																	//variable to gather load cell Fx,Fy,Fz,Mx,My,Mz
	float loads_delta_converted[LOAD_CELL_DOF] = {};														//holds the delta loads after they have conversion factor applied
	char zero = 'n';
	while (mhp->Moving){};
	lc.GetLoads(loads_msr);
	delay(2000);
	PrettyPrint::PrintLoad(loads_msr);
	vector<vector<double>> upper_deadbands = MHPSetUpperDeadbands(mhp);										//creates vector of upper deadbands

	//======== Initilize Load Cell ========//
	cout << "\nWould you like to connect load cell (y) or (n)?\n";
	cin >> zero;
	if (zero == 'y')
	{
		lc.Stop();
		fprintf(stdout, "\nStarting load cell\n\n");
		lc.Initialize(LOAD_CELL_CALIBRATION_PATH.c_str(),
			Config::LOAD_CELL_TRANSFORMATION,
			Config::SAMPLE_RATE,
			Config::LOADCELL_CHANNEL);
		delay(2000);
		lc.SetBias();
		delay(2000);
		//initializing load cell code

		//=========== Zero Load Cell ==========//
		lc.GetLoads(loads_msr);
		while ((loads_msr[0] > 0.1 || loads_msr[0] < -0.1) ||
			(loads_msr[1] > 0.1 || loads_msr[1] < -0.1) ||
			(loads_msr[2] > 0.1 || loads_msr[2] < -0.1) ||
			(loads_msr[3] > 0.1 || loads_msr[3] < -0.1) ||
			(loads_msr[4] > 0.1 || loads_msr[4] < -0.1) ||
			(loads_msr[5] > 0.1 || loads_msr[5] < -0.1))
		{
			delay(200);
			lc.SetBias();
			delay(200);
			lc.GetLoads(loads_msr);
		}																										//sets bias of load cell, zeroing it after platform is at zero point
		cout << "\nLoad Cell Zeroed\n";
		delay(2000);
		PrettyPrint::PrintLoad(loads_msr);
		lc.GetLoads(load_cell_zero);
		lc.GetLoads(load_cell_values);
		lc.GetLoads(load_cell_pid);
		lc.GetLoads(load_cell_current);
		lc.GetLoads(load_cell_delta);
	}
	cout << "Press 'Esc' to get out of loop\n";
	cout << "Press 'Enter' to hold position\n";
	cout << "Press 'Spacebar' to record loads and position\n";
	cout << "Press 'z' to zero robot\n\n";

	/////////////////////// LOOP ///////////////////////
	while (1)
	{
		try																							// try/catch need to be inside while loop with the movement function in order to catch any positions that are...
		{																							//...outside the range and would normally cause an error and stop the program
			//========= Gets Current Position From MHP Converts it =========//
			variant_t pos_var = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
			vector<double> cur_pos = DoubleVariantToVector(pos_var);								//currents 2D variant to a useable 1D 16 term vector
			vector<double> pos_ = MHPTranspose(cur_pos);											//converts variant from Left handed to right handed coordinate system, need to send variable and current loads
			current_pose_matrix <<																	//switches normal T matrix to eigen matrix to be able to use matrix functions (ie. matrix multiplication)
				pos_[0], pos_[1], pos_[2], pos_[3],
				pos_[4], pos_[5], pos_[6], pos_[7],
				pos_[8], pos_[9], pos_[10], pos_[11],
				0, 0, 0, 1;
			if (_kbhit())
			{
				int stop_char = _getch();
				if (stop_char == 122)																//'z' push
				{
					mhp->MovePosition(VARIANT_TRUE, L"Zero");										//Zeros MHP
					while (mhp->Moving)																//Loop to prevent any commands until robot is done moving
					{
					}
					std::cout << "MHP Zeroed!\n\n";
				}
				if (stop_char == 27)																//Esc command to get out of loop
				{
					mhp->Stop();
					cout << "\nLoop Stopped\n\n";
					break;
				}																					//this two if statements check if escape is pressed, if it is it will stop the robot, basically and estop
				if (stop_char == 32)																//'Spacebar' push
				{
					lc.GetLoads(load_cell_values);
					for (int z = 0; z < 6; z++)//added
					{
						load_vec[z] = static_cast<double>(load_cell_values[z]);
					}
					variant_t pos_var1 = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
					vector<double> cur_pos1 = DoubleVariantToVector(pos_var1);								//currents 2D variant to a useable 1D 16 term vector
					vector<double> pos1 = MHPTranspose(cur_pos1);											//transposes matrix tp conventional T matrix 
					vector<double> xyz = { pos1[3], pos1[7], pos1[11] };
					loads1.push_back(load_vec);//added
					position.push_back(xyz);
					cout << "\n";
					PrettyPrint::PrintLoad(load_cell_values);												//function to print load values on screen
					cout << "\n" << pos1[3] << ", " << pos1[7] << ", " << pos1[11] << "\n";
					cout << "\n";
					cout << "\nPress 'Spacebar' to record loads and position";
					cout << "\nPress 'Esc' to exit";
					cout << "\nPress 'z' to zero robot\n";
				}
				if (stop_char == 13)																//Enter key is hit
				{
					mhp->Stop();
					cout << "\nPress 'Spacebar' to record loads and position";
					cout << "\nPress 'Esc' to exit 'hold position'";
					cout << "\nPress 'z' to zero robot\n";

					while (1)
					{
						stop_char = _getch();
						if (stop_char == 27)														//Esc command to exit loop
						{
							cout << "\nHold Position Exited\n\n";
							cout << "\n=============================================\n";
							cout << "Press 'Esc' to get out of loop\n";
							cout << "Press 'Enter' to hold position\n";
							cout << "Press 'Spacebar' to record loads and position\n";
							cout << "Press 'z' to zero robot\n";
							break;
						}
						if (stop_char == 32)														//Spacebar is hit
						{
							lc.GetLoads(load_cell_values);
							for (int z = 0; z < 6; z++)//added
							{
								load_vec[z] = static_cast<double>(load_cell_values[z]);
							}
							variant_t pos_var1 = mhp->GetPositionMatrix(PositionActual, L"Reamer", L"Reamer Zero");	//gets a variant that is 2D of current position
							vector<double> cur_pos1 = DoubleVariantToVector(pos_var1);								//currents 2D variant to a useable 1D 16 term vector
							vector<double> pos1 = MHPTranspose(cur_pos1);											//transposes matrix tp conventional T matrix 
							vector<double> xyz = { pos1[3], pos1[7], pos1[11] };
							loads1.push_back(load_vec);//added
							position.push_back(xyz);
							cout << "\n";
							PrettyPrint::PrintLoad(load_cell_values);												//function to print load values on screen
							cout << "\n" << pos1[3] << ", " << pos1[7] << ", " << pos1[11] << "\n";
							cout << "\n";
							cout << "\nPress 'Spacebar' to record loads and position";
							cout << "\nPress 'Esc' to exit 'hold position'";
							cout << "\nPress 'z' to zero robot\n";
						}
						if (stop_char == 122)														//'z' push
						{
							mhp->MovePosition(VARIANT_TRUE, L"Zero");								//Zeros MHP
							while (mhp->Moving)														//Loop to prevent any commands until robot is done moving
							{
							}
							std::cout << "MHP Zeroed!\n\n";
						}
					}
				}
			}
			//========== Loads Difference and Proportial Control ==========//
			lc.GetLoads(load_cell_values);															//Gets loads Fx, Fy, Fz, Mx, My, Mz and stores in load_cell_values
			for (int z = 0; z < 6; z++)
			{
				load_cell_delta[z] = load_cell_values[z] - load_cell_zero[z];						//calculates a delta loads from the initial zero
			}

			load_cell_delta[3] = 0;
			load_cell_delta[4] = 0;
			load_cell_delta[5] = 0;

			for (int d = 0; d < 6; d++)
			{
				if (load_cell_delta[d] > deadbands[d][0] && load_cell_delta[d] < deadbands[d][1])	//checks if the adjusted delta load cell values are within the deadband values
				{
					load_cell_delta[d] = 0;															//sets value to zero within a range to prevent jittering
				}
			}
			//PrettyPrint::PrintLoad(load_cell_values);
			for (int p = 0; p < 6; p++)
			{
				load_cell_pid[p] = load_cell_delta[p] * p_control[p];								//proportial controller
			}

			//======== Check Upper Deadband =========//
			upper_deadbands = MHPSetUpperDeadbands(mhp);
			for (int term = 0; term < 6; term++)
			{
				if (load_cell_pid[term] < 0
					&& load_cell_pid[term] < upper_deadbands[term][0] * 0.15)										//checks if value is negative and > max neg deadband creating a threshold limit
				{
					load_cell_pid[term] = upper_deadbands[term][0] * 0.15;											//sets value to a max step size, limits correction
				}
				if (load_cell_pid[term] > 0
					&& load_cell_pid[term] > upper_deadbands[term][1] * 0.15)										//checks if value is negative and > max neg deadband creating a threshold limit
				{
					load_cell_pid[term] = upper_deadbands[term][1] * 0.15;											//sets value to a max step size, limits correction
				}
			}

			//================ Load Cell Transform Matrix ================//
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> lc_matrix;
			float sA = static_cast<float>(sin(RAD(load_cell_pid[5])));								//Rz coverted to alpha angle
			float sB = static_cast<float>(sin(RAD(load_cell_pid[4])));								//Ry coverted to beta angle
			float sG = static_cast<float>(sin(RAD(load_cell_pid[3])));								//Rx coverted to gamma angle
			float cA = static_cast<float>(cos(RAD(load_cell_pid[5])));								//Rz coverted to alpha angle
			float cB = static_cast<float>(cos(RAD(load_cell_pid[4])));								//Ry coverted to beta angle
			float cG = static_cast<float>(cos(RAD(load_cell_pid[3])));								//Rx coverted to gamma angle
			lc_matrix <<
				cA*cB, (cA*sB*sG) - (sA*cG), (cA*sB*cG) + (sA*sG), load_cell_pid[0],
				sA*cB, (sA*sB*sG) + (cA*cG), (sA*sB*cG) - (cA*sG), load_cell_pid[1],
				-sB, cB*sG, cB*cG, load_cell_pid[2],
				0, 0, 0, 1;																			//load cell T matrix
			//====== Matrix Multiplication To Base Coordinate System =====//
			final_pose_matrix = current_pose_matrix * lc_matrix;									//result pose calculation, mutliplcation of current and load cell T matrix

			//======== Convert Transform Calculation into Variant ========//
			for (unsigned int i = 0; i < 16; ++i) {
				next_pose_vec[i] = final_pose_matrix((i / 4), (i % 4));								//assign result pose back to vector
			}
			vector<double> next_pose_vec_trans = MHPTranspose(next_pose_vec);						//transposes vector so can be sent to MHP using its conventions
			variant_t next_pose_var = VectorToVariant(next_pose_vec_trans);							//converts vector to variant to be sent to MHP

			//======= Check if adjusted load cell are within Limits ======//																						//checks if any of the controls are non-zero
			if (load_cell_pid[0] != 0 || load_cell_pid[1] != 0 || load_cell_pid[2] != 0 ||
				load_cell_pid[3] != 0 || load_cell_pid[4] != 0 || load_cell_pid[5] != 0)
			{
				mhp->MoveMatrix(VARIANT_TRUE, L"Platform", L"Platform Zero", next_pose_var);		//moves the platform based on the matrix sent to it
			}
			else
			{
				mhp->Stop();
			}
		}
		catch (const _com_error& ex)
		{
			std::cout << "Position is Out of Range!\n";
			mhp->Stop();
		}
	}
	int size1 = loads1.size();
	string save_paths1 = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/digitize/" + file_name + "_coordinates.csv";
	ofstream savefile1(save_paths1);																//loops through each load file name to separate the loads
	for (int row = 0; row < size1; row++)																//used to loop through all the vectors 
	{
		for (int column = 0; column <= 2; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile1 << position[row][column];											//writes the specified element to the file
			if (column != 2)
			{
				savefile1 << ", ";																		//writes comma after each value in the file
			}
		}
		savefile1 << "\n";																				//writes a new line in the file onces a vector is completed
	};
	string save_loads = "C:/Users/HMMS/Documents/GitHub/Thesis/KUKA-LWR/MHP/digitize/" + file_name + "_loads.csv";
	ofstream savefile2(save_loads);																//loops through each load file name to separate the loads
	for (int row = 0; row < size1; row++)																//used to loop through all the vectors 
	{
		for (int column = 0; column <= 5; column++)														//used to loop through each element in the specified variable Fx,Fy,Fz,Mx,My,Mz
		{
			savefile2 << loads1[row][column];											//writes the specified element to the file
			if (column != 5)
			{
				savefile2 << ", ";																		//writes comma after each value in the file
			}
		}
		savefile2 << "\n";																				//writes a new line in the file onces a vector is completed
	};

	//======= Making Coordinate System ========//
	vector<double> origin = { position[0][0], position[0][1], position[0][2] };
	vector<double> Xneg = { position[1][0], position[1][1], position[1][2] };
	vector<double> Yneg = { position[2][0], position[2][1], position[2][2] };

	vector<double> XO = { -(Xneg[0] - origin[0]), -(Xneg[1] - origin[1]), -(Xneg[2] - origin[2]) };				//create Xneg vector
	vector<double> YO = { -(Yneg[0] - origin[0]), -(Yneg[1] - origin[1]), -(Yneg[2] - origin[2]) };				//create Yneg vector
	vector<double> ZO = { (ZO[1] * XO[2]) - (ZO[2] * XO[1]),
						  (ZO[2] * XO[0]) - (ZO[0] * XO[2]),
						  (ZO[0] * XO[1]) - (ZO[1] * XO[0]) };													//cross product X with Y to get Z
	vector<double> YOnew = { (XO[1] * YO[2]) - (XO[2] * YO[1]),
							 (XO[2] * YO[0]) - (XO[0] * YO[2]),
							 (XO[0] * YO[1]) - (XO[1] * YO[0]) };												//cross product Z with X to get new Y
	vector<double> T_cal = { XO[0], YO[0], ZO[0], origin[0],
							 XO[1], YO[1], ZO[1], origin[1],
							 XO[2], YO[2], ZO[2], origin[2],
								 0,		0,	   0,		  1,};
	return T_cal;
}
//============ MHP Ask and Set Velocity of Robot ===========//
void MHPVelocity(IHexapodPtr& mhp, Nano25E& lc)
{
	double velocity = 1;
	cout << "\nSet relative velocity between 0.00 - 1.00 (default = 1)\n";
	cin >> velocity;
	mhp->PutVelocity(velocity);
}
//=================== MHP ID Controller ===================//
void MHP_ID_Controller(vector<double> time, vector<vector<double>> error_all, vector<double>& integral, vector<double>& derivative)
{
	double iteration_time;
	vector<double> error_previous = { 0, 0, 0, 0, 0, 0 };
	vector<double> error_current = { 0, 0, 0, 0, 0, 0 };
	vector<double> error_delta = { 0, 0, 0, 0, 0, 0 };
	int time_length = time.size();
	double integral_array[10][6] = { 0 };

	//10 or less terms for integral term
	if (time_length == 1)
	{
		iteration_time = 0;																			//for first time loop
	}
	if (time_length > 1)
	{
		iteration_time = time[time_length - 1] - time[time_length - 2];								//current delta time
	}
	
	//========== Unfilled Array (<10) ==========//
	if (time_length <= 10)																			//sum iteration time for under 10 loops (1-10)
	{
		//========== Past Error ==========//
		for (int term = 0; term <= 5; term++)													//cycles through each term (x,y,z,Rx,Ry,Rz)
		{
			error_previous[term] = error_all[time_length - 2][term];							//summing up past error
		}
		//========== Current Error ==========//
		for (int term = 0; term <= 5; term++)														//cycles through each term (x,y,z,Rx,Ry,Rz)
		{
			error_current[term] = error_all[time_length-1][term];									//getting current error
		}
		//========== Delta Error ==========//
		for (int term = 0; term <= 5; term++)														//cycles through each term (x,y,z,Rx,Ry,Rz)
		{
			error_delta[term] = error_current[term] - error_previous[term];								//getting current error
		}
		//========== Derivative Calculation ==========//
		if (iteration_time != 0)
		{
			for (int term = 0; term <= 5; term++)														//cycles through each term (x,y,z,Rx,Ry,Rz)
			{
				derivative[term] = error_delta[term] / iteration_time;								//getting current error
			}
		}
		//========== Integral Calculations ==========//
		for (int j = 0; j <= 5; j++)																//sums up all past times except most recent
		{
			integral_array[time_length - 1][j] = error_all[time_length - 1][j] * iteration_time;			//integral term calculation
		}
		for (int k = time_length-1; k >= 0; k--)
		{
			for (int term = 0; term <= 5; term++)													//cycles through each term (x,y,z,Rx,Ry,Rz)
			{
				integral[term] = integral_array[k][term] + integral[term];
			}
		}
	}

	//10 or more terms for integral term
	if (time_length > 10)
	{
		//========== Integral Moving Window ==========//
		for (int a = 0; a < 10; a++)
		{
			for (int b = 0; b < 6; b++)
			{
				integral_array[a][b] = integral_array[a+1][b];													//move up each row in array to open last row for new calculation
			}
		}
		//========== Most recent Integral Row ==========//
		for (int j = 0; j <= 5; j++)																//sums up all past times except most recent
		{
			integral_array[time_length - 1][j] = error_all[time_length - 1][j] * iteration_time;			//integral term calculation to last row of array
		}
		//========== Summing Past Integral Terms ==========//
		for (int k = 9; k >= 0; k--)																//add each of 10 rows of integral moving window
		{
			for (int term = 0; term <= 5; term++)
			{
				integral[term] = integral_array[k][term] + integral[term];						//integral sum error for past 10 terms
			}
		}
		//========== Past Error ==========//
		for (int term = 0; term <= 5; term++)													//cycles through each term (x,y,z,Rx,Ry,Rz)
		{
			error_previous[term] = error_all[time_length - 2][term];							//summing up past error
		}
		//========== Current Error ==========//
		for (int term = 0; term <= 5; term++)														//cycles through each term (x,y,z,Rx,Ry,Rz)
		{
			error_current[term] = error_all[time_length - 1][term];									//getting current error
		}
		//========== Delta Error ==========//
		for (int term = 0; term <= 5; term++)														//cycles through each term (x,y,z,Rx,Ry,Rz)
		{
			error_delta[term] = error_current[term] - error_previous[term];								//getting current error
		}
		//========== Derivative Calculation ==========//
		if (iteration_time != 0)
		{
			for (int term = 0; term <= 5; term++)														//cycles through each term (x,y,z,Rx,Ry,Rz)
			{
				derivative[term] = error_delta[term] / iteration_time;								//getting current error
			}
		}
	}
}