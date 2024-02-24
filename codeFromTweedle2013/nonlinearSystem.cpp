#include "nonlinearSystem.h"

nonlinearSystem::nonlinearSystem() { h = DEFAULT_H; }
VectorXd nonlinearSystem::propagateRK4(double tf, VectorXd x0) {
  VectorXd k1;
  VectorXd k2;
  VectorXd k3;
  VectorXd k4;
  double t = 0;
  double dt;
  bool done = false;
  VectorXd x = x0;  // std::cout << "x(" << t << "): " << x.transpose() << std::endl;
  while (!done) {
    if (tf - h - t > 0) {
      dt = h;
    } else {
      dt = tf - t;
      done = true;
    }
    k1 = dt * this->f(x);
    k2 = dt * this->f(x + 0.5 * k1);
    k3 = dt * this->f(x + 0.5 * k2);
    k4 = dt * this->f(x + k3);
    x = x + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    t += dt;
    // std::cout << "x(" << t << "): " << x.segment<12>(0).transpose() << std::endl;
  }
  return x;
}

VectorXd nonlinearSystem::propagateRK4_adaptive(double tf, VectorXd x0) {
  bool done = false;
  double h_starting = this->h;
  this->h = tf / INITIAL_H_FACTOR;

  VectorXd newX, errX;
  VectorXd currX = this->propagateRK4(tf, x0);

  while (!done) {
    // new h step size
    this->h = this->h / 2;
    // try the new step size
    newX = this->propagateRK4(tf, x0);
    // compute rms error 58 errX = newX - currX;
    double rms_err = sqrt(errX.squaredNorm() / errX.size());
    // std::cout <<"propagateRK4 Adaptive, h=" << h << ", rms_err=" << rms_err << std::endl;

    // check rms_error or if h is too small that it will take too long
    if (rms_err < RMS_ERR_CUTOFF || this->h <= (tf / LOWER_H_LIMIT_FACTOR)) {
      done = true;
      if (this->h <=
          (tf / LOWER_H_LIMIT_FACTOR)) {  // std::cout << "adaptive RK4 timestep was cutoff"
                                          // << std::endl;
      }
    } else {
      currX = newX;
    }
  }
  this->h = h_starting;
  return newX;
}