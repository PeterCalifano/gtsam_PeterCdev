#ifndef NONLINEARSYSTEM_H_
#define NONLINEARSYSTEM_H_

#include <math.h>
#include <Eigen/Dense>
#include <iostream>

#define DEFAULT_H 0.05
#define LOWER_H_LIMIT_FACTOR 20
#define RMS_ERR_CUTOFF 1.0e-5
#define INITIAL_H_FACTOR 1

using namespace Eigen;
class nonlinearSystem {
  double h;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  nonlinearSystem();
  VectorXd propagateRK4(double tf, VectorXd x0);
  VectorXd propagateRK4_adaptive(double tf, VectorXd x0);
  void setStepSize(double _h) { h = _h; };
  virtual VectorXd f(VectorXd x) = 0;
};

#endif