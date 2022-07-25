namespace BoostMatrix {

  void BuildPoseArrayFromMatrix(boost::numeric::ublas::matrix<float>& mat,
    float* pose);

  void BuildPoseMatrixFromArray(
    const float(&pose_arr)[NUMBER_OF_FRAME_ELEMENTS],
    boost::numeric::ublas::matrix<float>* mat);

  void BuildMatrixFromArray(const int width, const int height, float* values,
    int values_size, boost::numeric::ublas::matrix<float>* mat);

  void BuildMatrixFromVector(const int width, const int height,
    std::vector<float>* values, boost::numeric::ublas::matrix<float>* mat);

  template<class T, std::size_t N>
  T(&as_simple_array(boost::array<T, N>& arr))[N] {
    return *reinterpret_cast<T(*)[N]>(arr.data());
  }
  void BuildPoseFromRotationMatrix(const Eigen::Matrix3d& mat, float(&pose)[12]);

  //void GetXYZEulerRotationsFromPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], position& rot);
  void GetXYZEulerRotationsFromPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], Eigen::Vector3d& rot);
};

namespace BoostMatrix {

  void BuildPoseArrayFromMatrix(matrix<float>& mat, float* pose) {
    for (unsigned int i = 0; i < mat.size1() - 1; ++i) {
      for (unsigned int j = 0; j < mat.size2(); ++j) {
        pose[mat.size2() * i + j] = mat(i, j);
      }
    }
  }

  void BuildMatrixFromVector(const int width, const int height,
    vector<float>* values, matrix<float>* mat) {
    int i = 0;
    int j = 0;
    for (std::vector<float>::iterator it = values->begin(); it != values->end(); ++it) {
      mat->insert_element(i++ / width, j++%width, *it);
    }
  }

  void BuildMatrixFromArray(int width, int height, float* values,
    int values_size, matrix<float>* mat) {
    int i = 0;
    for (int i = 0; i < values_size; ++i) {
      mat->insert_element(i / width, i%width, values[i]);
    }
  }

  void BuildPoseMatrixFromArray(const float(&pose_arr)[NUMBER_OF_FRAME_ELEMENTS],
    matrix<float>* mat) {
    int j, i = 0;
    for (j = 0; j < NUMBER_OF_FRAME_ELEMENTS; ++j) {
      mat->insert_element(i, j % 4, pose_arr[j]);
      if ((j + 1) % 4 == 0) {
        i++;
      }
    }

    mat->insert_element(3, 0, 0);
    mat->insert_element(3, 1, 0);
    mat->insert_element(3, 2, 0);
    mat->insert_element(3, 3, 1);
  }

  /*
  void GetXYZEulerRotationsFromPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], position& rot) {
    Matrix3f m;
    m << pose[0], pose[1], pose[2],
      pose[4], pose[5], pose[6],
      pose[8], pose[9], pose[10];

    Vector3f ea = m.eulerAngles(1, 0, 2); // y,x,z  // fixed z,x,y

    rot.x = static_cast<float>(DEG(ea[0]));
    rot.y = static_cast<float>(DEG(ea[1]));
    rot.z = static_cast<float>(DEG(ea[2]));
  }
  */

  void GetXYZEulerRotationsFromPose(const float(&pose)[NUMBER_OF_FRAME_ELEMENTS], Eigen::Vector3d& rot) {
    Matrix3f m;
    m << pose[0], pose[1], pose[2],
      pose[4], pose[5], pose[6],
      pose[8], pose[9], pose[10];

    rot = m.eulerAngles(1, 0, 2); // y,x,z  // fixed z,x,y
  }

  void BuildPoseFromRotationMatrix(const Eigen::Matrix3d& mat, float(&pose)[12]) {
    pose[0] = mat(0, 0);
    pose[1] = mat(0, 1);
    pose[2] = mat(0, 2);
    pose[4] = mat(1, 0);
    pose[5] = mat(1, 1);
    pose[6] = mat(1, 2);
    pose[8] = mat(2, 0);
    pose[9] = mat(2, 1);
    pose[10] = mat(2, 2);
  }

  /*
  void GetRotationMatrixFromEulerAngles(float x, float y, float z) {
    Matrix3f n;
    n = AngleAxisf(ea[0], Vector3f::UnitX())
      *AngleAxisf(ea[1], Vector3f::UnitY())
      *AngleAxisf(ea[2], Vector3f::UnitZ());
    cout << "recalc original rotation:" << endl;
    cout << n << endl;
  }
  */
}  // end namespace