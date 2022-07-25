#include "PID6DOF.h"
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
using Eigen::Matrix3f;

// defaults to position, rotation
PID6DOF::PID6DOF() {

  //x_->setInputLimits(-75, 75);  // force
  //y_->setInputLimits(-75, 75);
  //z_->setInputLimits(-75, 75);
  //a_->setInputLimits(-5, 5);  // torque
  //b_->setInputLimits(-5, 5);  
  //c_->setInputLimits(-5, 5);

  x_.setInputLimits(-0.8, 0.8);  // force
  y_.setInputLimits(-0.8, 0.8);
  z_.setInputLimits(-0.8, 0.8);
  a_.setInputLimits(-1, 1);  // torque
  b_.setInputLimits(-1, 1);
  c_.setInputLimits(-1, 1);

  x_.setOutputLimits(-0.0006, 0.0006);  // translation (mm/step)
  y_.setOutputLimits(-0.0006, 0.0006);
  z_.setOutputLimits(-0.0006, 0.0006);
  a_.setOutputLimits(-0.001, 0.001); // rotation (rad/step)
  b_.setOutputLimits(-0.001, 0.001);
  c_.setOutputLimits(-0.001, 0.001);
}

void PID6DOF::setGains(const float (&gains)[6]) {
  x_.setTunings(gains[0], x_.getIParam(), x_.getDParam());
  y_.setTunings(gains[1], y_.getIParam(), y_.getDParam());
  z_.setTunings(gains[2], z_.getIParam(), z_.getDParam());
  a_.setTunings(gains[3], a_.getIParam(), a_.getDParam());
  b_.setTunings(gains[4], b_.getIParam(), b_.getDParam());
  c_.setTunings(gains[5], c_.getIParam(), c_.getDParam());
}

void PID6DOF::setIntegrals(const float(&integrals)[6]) {
  x_.setTunings(x_.getPParam(), integrals[0], x_.getDParam());
  y_.setTunings(y_.getPParam(), integrals[1], y_.getDParam());
  z_.setTunings(z_.getPParam(), integrals[2], z_.getDParam());
  a_.setTunings(a_.getPParam(), integrals[3], a_.getDParam());
  b_.setTunings(b_.getPParam(), integrals[4], b_.getDParam());
  c_.setTunings(c_.getPParam(), integrals[5], c_.getDParam());
}

void PID6DOF::setInputLimits(const unsigned int num, const int low, const int high) {
  switch (num) {
  case 0:
    x_.setInputLimits(low, high);
    break;
  case 1:
    y_.setInputLimits(low, high);
    break;
  case 2:
    z_.setInputLimits(low, high);
    break;
  case 3:
    a_.setInputLimits(low, high);
    break;
  case 4:
    b_.setInputLimits(low, high);
    break;
  case 5:
    c_.setInputLimits(low, high);
    break;
  default:
    printf("Inproper selection");
  }
}

void PID6DOF::setSetPoint(const float(&dsr)[12]) {
  Matrix3f m_dsr;
  m_dsr <<
    dsr[0], dsr[1], dsr[2],
    dsr[4], dsr[5], dsr[6],
    dsr[8], dsr[9], dsr[10];
  Eigen::Quaternionf q_dsr(m_dsr);

  x_.setSetPoint(dsr[3]);
  y_.setSetPoint(dsr[7]);
  z_.setSetPoint(dsr[11]);
  a_.setSetPoint(q_dsr.x());
  b_.setSetPoint(q_dsr.y());
  c_.setSetPoint(q_dsr.z());
}

void PID6DOF::setProcessValue(const float (&msr)[12]) {
  Matrix3f m_msr;
  m_msr <<
    msr[0], msr[1], msr[2],
    msr[4], msr[5], msr[6],
    msr[8], msr[9], msr[10];
  Eigen::Quaternionf q_msr(m_msr);

  x_.setProcessValue(msr[3]);
  y_.setProcessValue(msr[7]);
  z_.setProcessValue(msr[11]);
  a_.setProcessValue(q_msr.x());
  b_.setProcessValue(q_msr.y());
  c_.setProcessValue(q_msr.z());
}

void PID6DOF::compute(float* next) {
  Matrix3f m_next;
  m_next <<
    next[0], next[1], next[2],
    next[4], next[5], next[6],
    next[8], next[9], next[10];
  Eigen::Quaternionf q_next(m_next);

  q_next.x() += a_.compute();
  q_next.y() += b_.compute();
  q_next.z() += c_.compute();
  q_next.normalize();
  m_next = q_next.toRotationMatrix();

  next[0] = m_next(0, 0);
  next[1] = m_next(0, 1);
  next[2] = m_next(0, 2);
  next[3] += x_.compute();
  next[4] = m_next(1, 0);
  next[5] = m_next(1, 1);
  next[6] = m_next(1, 2);
  next[7] += y_.compute();
  next[8] = m_next(2, 0);
  next[9] = m_next(2, 1);
  next[10] = m_next(2, 2);
  next[11] += z_.compute();

  //printf("%.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n", x_.compute(), next[3], y_.compute(), next[7], z_.compute(), next[11]);
}