  void CrossProduct(float rx, float ry, float rz, float fx, float fy, float fz,
    float* mx, float* my, float* mz) {
    *mx = (ry*fz) - (rz*fy);
    *my = (rz*fx) - (rx*fz);
    *mz = (rx*fy) - (ry*fx);
  }

  string PoseToString(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS]) {
    string s = "";
    for (int i = 0; i < 12; ++i) {
      s.append(std::to_string(pose[i]));
      if ((i + 1) % 4 == 0) {
        s.append("\n");
      }
      else {
        s.append(", ");
      }
    }
    return s;
  }

  bool ReachedAcceptableLoadPosition(load& val, load& val_desired,
    load& diff_threshold) {

    float lax_threshold = 0.3;

    int count = 0;
    int lax_count = 0;
    float err_x = abs(val_desired.x - val.x);
    float err_y = abs(val_desired.y - val.y);
    float err_z = abs(val_desired.z - val.z);
    float err_mx = abs(val_desired.mx - val.mx);
    float err_my = abs(val_desired.my - val.my);
    float err_mz = abs(val_desired.mz - val.mz);

    err_x < diff_threshold.x ? count++ : err_x < lax_threshold ? lax_count++ : lax_count; 
    err_y < diff_threshold.y ? count++ : err_y < lax_threshold ? lax_count++ : lax_count;
    err_z < diff_threshold.z ? count++ : err_z < lax_threshold ? lax_count++ : lax_count;
    err_mx < diff_threshold.mx ? count++ : err_mx < lax_threshold ? lax_count++ : lax_count;
    err_my < diff_threshold.my ? count++ : err_my < lax_threshold ? lax_count++ : lax_count;
    err_mz < diff_threshold.mz ? count++ : err_mz < lax_threshold ? lax_count++ : lax_count;

    PrettyPrint::PrintAbsoluteDifference(val_desired, val);

    return (count == 5 && lax_count == 1) || (count == 6);
  }
  void SetDesiredPosition(float(&load_cell_forces)[6], load& find_load) {
    load_cell_forces[0] *= -1;
    load_cell_forces[0] += find_load.x;
    load_cell_forces[1] *= -1;
    load_cell_forces[1] += find_load.y;
    load_cell_forces[2] *= -1;
    load_cell_forces[2] += find_load.z;
    load_cell_forces[3] *= -1;
    load_cell_forces[3] += find_load.mx;
    load_cell_forces[4] *= -1;
    load_cell_forces[4] += find_load.my;
    load_cell_forces[5] *= -1;
    load_cell_forces[5] += find_load.mz;
  }

  void SetScalingFactor(float(&scale)[6], load& scaling_factor) {
    scale[0] = scaling_factor.x;
    scale[1] = scaling_factor.y;
    scale[2] = scaling_factor.z;
    scale[3] = scaling_factor.mx;
    scale[4] = scaling_factor.my;
    scale[5] = scaling_factor.mz;
  }