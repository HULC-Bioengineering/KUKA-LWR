  /// DH parameters of the robot
  static const double D1;
  static const double D3;
  static const double D5;
  static const double D7;
  static const double ALPHA;
  
  // rotation limits of each joint (rad)
  static const double JNT_LIMITS [];

  static bool ikSolver(std::vector<double>& jntPosMsr, double(&pose)[12],
    std::vector<double>& jntPosDsr);

  static bool fkSolver(const std::vector<double> & jntPosDsr,
    double(&dsr_pose)[12]);

  static void DH(double a, double alpha, double d, double theta,
    Eigen::Matrix4d& FXX);

// DH parameters
const double LWR::D1 = 0.310;
const double LWR::D3 = 0.400;
const double LWR::D5 = 0.390;
const double LWR::D7 = 0.078;
const double LWR::ALPHA = PI / 2.0;
const double LWR::JNT_LIMITS[] = {
  170.0*PI / 180.0,
  120.0*PI / 180.0,
  170.0*PI / 180.0,
  120.0*PI / 180.0,
  170.0*PI / 180.0,
  120.0*PI / 180.0,
  170.0*PI / 180.0 
};

///////////////////////////////////////////////////////////////////////////////
////                             FK & IK                                   ////
///////////////////////////////////////////////////////////////////////////////
bool LWR::ikSolver(std::vector<double>& jntPosMsr, double(&dsr_pose)[12],
  std::vector<double>& jntPosDsr) {

  bool output = true;  // return value
  jntPosDsr = std::vector<double>(7, 0.0);  // initialize

  // Auxiliar variables
  double mod_pW, mod_pWxy, c2, s2, c3, s3;

  // Getting transformation matrix
  Matrix3d R;
  R << dsr_pose[0], dsr_pose[1], dsr_pose[2],
    dsr_pose[4], dsr_pose[5], dsr_pose[6],
    dsr_pose[8], dsr_pose[9], dsr_pose[10];

  Vector3d Runitz;
  Runitz << dsr_pose[2], dsr_pose[6], dsr_pose[10];

  Vector3d p;
  p << dsr_pose[3], dsr_pose[7], dsr_pose[11];

  // Inverse kinematics calculations. Closed form solution.
  // Based in the solution for an anthropomorfic arm with an spherical wrist.
  // Extracted from Siciliano et al (2010)
  // wrist position
  Vector3d pW;
  pW = p - D7 * Runitz;

  // Calculate wrist position
  jntPosDsr[0] = std::atan2(pW[1], pW[0]);
  mod_pW = pow(pW.norm(), 2);

  c3 = (mod_pW - D3*D3 - D5*D5) / (2 * D3*D5);
  // If c3>1, there is no solution for IKT
  if (c3>1){
    output = false;
    cout << "ikSolver: Attempt to access to a point out of the workspace. Zero array will be returned." << endl;
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
  jntPosDsr[5] = atan2(sqrt(pose36(0, 2)*pose36(0, 2) + pose36(1, 2)*pose36(1, 2)), pose36(2, 2));
  jntPosDsr[6] = atan2(pose36(2, 1), -pose36(2, 0));

  //Adjust to robot from IK coordinates (keeping joint coord. within the interval [-pi,pi])
  jntPosDsr[1] < -PI / 2 ? jntPosDsr[1] += 3 * PI / 2 : jntPosDsr[1] -= PI / 2;
  jntPosDsr[3] < -PI / 2 ? jntPosDsr[3] += 3 * PI / 2 : jntPosDsr[3] -= PI / 2;
  jntPosDsr[6] <     0 ? jntPosDsr[6] += PI : jntPosDsr[6] -= PI;

  for (int i = 0; i < (int)jntPosDsr.size(); i++){
    if (abs(jntPosDsr[i]) > JNT_LIMITS[i]){
      output = false;
      jntPosDsr[i] > 0 ? jntPosDsr[i] = JNT_LIMITS[i] : jntPosDsr[i] = -JNT_LIMITS[i];
      cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
    }
    DBGVAR(std::cout, jntPosDsr[i]);
  }
  return output;
}

bool LWR::fkSolver(const std::vector<double>& jntPosDsr, double(&dsr_pose)[12]) {
  // Local copy of the desired joint positions vector
  std::vector<double> jntPos(jntPosDsr);
  bool output = true;

  //Check requested angles are within the limits
  for (int i = 0; i < (int)jntPos.size(); i++){
    if (abs(jntPos[i]) > JNT_LIMITS[i]){
      jntPos[i] > 0 ? jntPos[i] = JNT_LIMITS[i] : jntPos[i] = -JNT_LIMITS[i];
      cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
      output = false;
    }
  }

  //Adjust to robot to FK coordinates from coordinates in the joint space
  jntPos[1] > 0 ? jntPos[1] -= PI : jntPos[1] += PI;
  jntPos[3] > 0 ? jntPos[3] -= PI : jntPos[3] += PI;
  jntPos[5] > 0 ? jntPos[5] -= PI : jntPos[5] += PI;
  jntPos[6] > 0 ? jntPos[6] -= PI : jntPos[6] += PI;

  Matrix4d F01; DH(0.0, ALPHA, D1, jntPos[0], F01);
  Matrix4d F12; DH(0.0, ALPHA, 0.0, jntPos[1], F12);
  Matrix4d F23; DH(0.0, ALPHA, D3, jntPos[2], F23);
  Matrix4d F34; DH(0.0, ALPHA, 0.0, jntPos[3], F34);
  Matrix4d F45; DH(0.0, ALPHA, D5, jntPos[4], F45);
  Matrix4d F56; DH(0.0, ALPHA, 0.0, jntPos[5], F56);
  Matrix4d F67; DH(0.0, 0.0, D7, jntPos[6], F67);

  Matrix4d F7tool;
  F7tool <<
    1, 0, 0, -0.078070,
    0, 1, 0, -0.001178,
    0, 0, 1,  0.132597,
    0, 0, 0,  1;

  // Forward Kinematics transformation
  Matrix4d F0tool;
  F0tool = F01*F12*F23*F34*F45*F56*F67*F7tool;

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
    st, ct*ca,  -ct*sa, a*st,
    0,  sa,     ca,    d,
    0,  0,      0,     1; 
}
