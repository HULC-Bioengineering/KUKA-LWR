/**
* @author Aaron Berk
*
* @section LICENSE
*
* Copyright (c) 2010 ARM Limited
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* @section DESCRIPTION
*
* A PID controller is a widely used feedback controller commonly found in
* industry.
*
* This library is a port of Brett Beauregard's Arduino PID library:
*
*  http://www.arduino.cc/playground/Code/PIDLibrary
*
* The wikipedia article on PID controllers is a good place to start on
* understanding how they work:
*
*  http://en.wikipedia.org/wiki/PID_controller
*
* For a clear and elegant explanation of how to implement and tune a
* controller, the controlguru website by Douglas J. Cooper (who also happened
* to be Brett's controls professor) is an excellent reference:
*
*  http://www.controlguru.com/
*/

/**
* Includes
*/
#include "PID.h"
#include <stdio.h>
PID::PID() {
  usingFeedForward = false;

  // 0.5 mm translatiom / step max. 500mm max away
  setInputLimits(-0.800, 0.800);
  setOutputLimits(-0.0005, 0.0005);

  tSample_ = 0.002;

  setTunings(0, 0, 0);

  setPoint_ = 0.0;
  processVariable_ = 0.0;
  prevProcessVariable_ = 0.0;
  controllerOutput_ = 0.0;
  prevControllerOutput_ = 0.0;

  accError_ = 0.0;
  bias_ = 0.0;

  realOutput_ = 0.0;
}

PID::PID(double Kc, double tauI, double tauD, double interval) {

  usingFeedForward = false;

  // 0.5 mm translatiom / step max. 500mm max away
  setInputLimits(-0.800, 0.800);
  setOutputLimits(-0.0005, 0.0005);

  tSample_ = interval;

  setTunings(Kc, tauI, tauD);

  setPoint_ = 0.0;
  processVariable_ = 0.0;
  prevProcessVariable_ = 0.0;
  controllerOutput_ = 0.0;
  prevControllerOutput_ = 0.0;

  accError_ = 0.0;
  bias_ = 0.0;

  realOutput_ = 0.0;
}

void PID::setInputLimits(double inMin, double inMax) {

  //Make sure we haven't been given impossible values.
  if (inMin >= inMax) {
    return;
  }

  //Rescale the working variables to reflect the changes.
  prevProcessVariable_ *= (inMax - inMin) / inSpan_;
  accError_ *= (inMax - inMin) / inSpan_;

  //Make sure the working variables are within the new limits.
  if (prevProcessVariable_ > 1) {
    prevProcessVariable_ = 1;
  }
  else if (prevProcessVariable_ < 0) {
    prevProcessVariable_ = 0;
  }

  inMin_ = inMin;
  inMax_ = inMax;
  inSpan_ = inMax - inMin;

}

void PID::setOutputLimits(double outMin, double outMax) {

  //Make sure we haven't been given impossible values.
  if (outMin >= outMax) {
    printf("Impossible");
    return;
  }

  //Rescale the working variables to reflect the changes.
  prevControllerOutput_ *= (outMax - outMin) / outSpan_;

  //Make sure the working variables are within the new limits.
  if (prevControllerOutput_ > 1) {
    prevControllerOutput_ = 1;
  }
  else if (prevControllerOutput_ < 0) {
    prevControllerOutput_ = 0;
  }

  outMin_ = outMin;
  outMax_ = outMax;
  outSpan_ = outMax - outMin;

}

void PID::setTunings(double Kc, double tauI, double tauD) {

  //Verify that the tunings make sense.
  if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0) {
    return;
  }

  //Store raw values to hand back to user on request.
  pParam_ = Kc;
  iParam_ = tauI;
  dParam_ = tauD;

  if (tauI == 0.0) {
    tauR_ = 0.0;
  }
  else {
    tauR_ = (1.0 / tauI) * tSample_;
  }

  //For "bumpless transfer" we need to rescale the accumulated error.
  // # edited by @matthewbstokes to remove auto  mode
    if (tauR_ == 0.0) {
      accError_ = 0.0;
    }
    else {
      accError_ *= (Kc_ * tauR_) / (Kc * tauR_);
    }

  Kc_ = Kc;
  tauD_ = tauD / tSample_;

}

void PID::reset(void) {

  double scaledBias = 0.0;

  if (usingFeedForward) {
    scaledBias = (bias_ - outMin_) / outSpan_;
  }
  else {
    scaledBias = (realOutput_ - outMin_) / outSpan_;
  }

  prevControllerOutput_ = scaledBias;
  prevProcessVariable_ = (processVariable_ - inMin_) / inSpan_;

  //Clear any error in the integral.
  accError_ = 0;

}

void PID::setInterval(double interval) {

  if (interval > 0) {
    //Convert the time-based tunings to reflect this change.
    tauR_ *= (interval / tSample_);
    accError_ *= (tSample_ / interval);
    tauD_ *= (interval / tSample_);
    tSample_ = interval;
  }

}

void PID::setSetPoint(double sp) {

  setPoint_ = sp;

}

void PID::setProcessValue(double pv) {

  processVariable_ = pv;

}

void PID::setBias(double bias){

  bias_ = bias;
  usingFeedForward = 1;

}

double PID::computeNonCenter() {
  //Pull in the input and setpoint, and scale them into percent span.
  printf("pV: %.5f ", processVariable_);
  double scaledPV = (processVariable_ - inMin_) / inSpan_;

  if (scaledPV > 1.0) {
    scaledPV = 1.0;
  }
  else if (scaledPV < 0) {
    scaledPV = 0;
  }

  printf("sP: %.5f ", setPoint_);
  double scaledSP = (setPoint_ - inMin_) / inSpan_;
  if (scaledSP > 1.0) {
    scaledSP = 1.0;
  }
  else if (scaledSP < 0) {
    scaledSP = 0;
  }

  double error = scaledSP - scaledPV;
  prev_error_ = error;
  //Check and see if the output is pegged at a limit and only
  //integrate if it is not. This is to prevent reset-windup.
  if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= 0 && error < 0)) {
    accError_ += error;
    //printf("\nacc: %.7f\n", accError_);
  }

  //Compute the current slope of the input signal.
  double dMeas = (processVariable_ - prevProcessVariable_) / tSample_; // (processVariable_ - prevProcessVariable_ <= inMin) // always

  double scaledBias = 0.0;

  //if (usingFeedForward) {
  //  scaledBias = (bias_ - outMin_) / outSpan_;
  //}

  //Perform the PID calculation.
  printf("e: %.5f ", error);
  controllerOutput_ = scaledBias + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));
  printf("cO: %.5f ", controllerOutput_);
  //printf("%.7f\t", tauR_ * accError_ * Kc_);
  //printf("%.7f\t", error * Kc_);

  //Make sure the computed output is within output constraints.
  if (controllerOutput_ < 0) {
    controllerOutput_ = 0;
  }
  else if (controllerOutput_ > 1.0) {
    controllerOutput_ = 1.0;
  }

  //Remember this output for the windup check next time.
  prevControllerOutput_ = controllerOutput_;
  //Remember the input for the derivative calculation next time.
  prevProcessVariable_ = processVariable_;

  prevComputed_ = controllerOutput_ * outSpan_ + outMin_;
  printf("c: %.5f\n", controllerOutput_ * outSpan_ + outMin_);
  return controllerOutput_ * outSpan_ + outMin_;
}


/*
double PID::computeNonCenter() {

  double error = processVariable_ - setPoint_;
  prev_error_ = error;
  //Check and see if the output is pegged at a limit and only
  //integrate if it is not. This is to prevent reset-windup.
  if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= -1 && error < 0)) {
    accError_ += error;
    //printf("\nacc: %.7f\n", accError_);
  }

  //Compute the current slope of the input signal.
  double dMeas = (processVariable_ - prevProcessVariable_) / tSample_; // (processVariable_ - prevProcessVariable_ <= inMin) // always

  double scaledBias = 0.0;

  if (usingFeedForward) {
    scaledBias = (bias_ - outMin_) / outSpan_;
  }

  //Perform the PID calculation.
  controllerOutput_ = scaledBias + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));
  //printf("%.7f\t", tauR_ * accError_ * Kc_);
  //printf("%.7f\t", error * Kc_);

  //Make sure the computed output is within output constraints.
  if (controllerOutput_ < 0) {
    controllerOutput_ = 0;
  }
  else if (controllerOutput_ > 1.0) {
    controllerOutput_ = 1.0;
  }

  //Remember this output for the windup check next time.
  prevControllerOutput_ = controllerOutput_;
  //Remember the input for the derivative calculation next time.
  prevProcessVariable_ = processVariable_;

  //Scale the output from percent span back out to a real world number.
  // assuming 0 centered
  prevComputed_ = arduino_map(controllerOutput_, 0, 1, outMin_, outMax_);
  return prevComputed_;
}
*/
// @matthewbstokes adjusted lower range to include negative values
double PID::compute() {

  //Pull in the input and setpoint, and scale them into percent span.
  double scaledPV = (processVariable_ - inMin_) / inSpan_;

  if (scaledPV > 1.0) {
    scaledPV = 1.0;
  }
  else if (scaledPV < -1.0) {
    scaledPV = -1.0;
  }

  double scaledSP = (setPoint_ - inMin_) / inSpan_;
  if (scaledSP > 1.0) {
    scaledSP = 1.0;
  }
  else if (scaledSP < -1.0) {
    scaledSP = -1.0;
  }

  double error = scaledSP - scaledPV;
  //error = setPoint_ - processVariable_;

  prev_error_ = error;
  //Check and see if the output is pegged at a limit and only
  //integrate if it is not. This is to prevent reset-windup.
  if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= -1 && error < 0)) {
    accError_ += error;
    //printf("\nacc: %.7f\n", accError_);
  }

  //Compute the current slope of the input signal.
  double dMeas = (processVariable_ - prevProcessVariable_) / tSample_; // (processVariable_ - prevProcessVariable_ <= inMin) // always

  double scaledBias = 0.0;

  if (usingFeedForward) {
    scaledBias = (bias_ - outMin_) / outSpan_;
  }

  //Perform the PID calculation.
  controllerOutput_ = scaledBias + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));
  //printf("%.7f\t", tauR_ * accError_ * Kc_);
  //printf("%.7f\t", error * Kc_);

  //Make sure the computed output is within output constraints.
  if (controllerOutput_ < -1.0) {
    controllerOutput_ = -1.0;
  }
  else if (controllerOutput_ > 1.0) {
    controllerOutput_ = 1.0;
  }

  //Remember this output for the windup check next time.
  prevControllerOutput_ = controllerOutput_;
  //Remember the input for the derivative calculation next time.
  prevProcessVariable_ = processVariable_;

  //Scale the output from percent span back out to a real world number.
  // assuming 0 centered
  prevComputed_ = controllerOutput_ * outSpan_ / 2;   // save computed value for reference
  return ((controllerOutput_ * outSpan_ / 2));  
}

double PID::getInMin() {
  return inMin_;
}

double PID::getInMax() {
  return inMax_;
}

double PID::getOutMin() {
  return outMin_;
}

double PID::getOutMax() {
  return outMax_;
}

double PID::getInterval() {
  return tSample_;
}

double PID::getPParam() {
  return pParam_;
}

double PID::getIParam() {
  return iParam_;
}

double PID::getDParam() {
  return dParam_;
}

double PID::getProcessValue() {
  return processVariable_;
}

double PID::getSetPoint() {
  return setPoint_;
}

double PID::getLastComputed() {
  return prevComputed_;
}

double PID::getAbsDiff() {
  return abs(setPoint_ - processVariable_);
}

double PID::getPrevousProcessValue() {
  return prevProcessVariable_;
}