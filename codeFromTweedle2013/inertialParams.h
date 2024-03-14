#pragma once
#include <isam/util.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <ostream>
#include "math.h"

using namespace isam;
using namespace Eigen;

class principalAxesFrame {
  std::ostream& operator<<(std::ostream& out, const principalAxesFrame& p) {
    p.write(out);
    return out;
  }
  Vector3d _r;  // position - from the target frame to the principal frame
  Vector4d _q;  // quaternion - from the target frame to the principal frame parameter attitude error
  Vector3d _a;  // Modified Rodrigues Parameter
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int dim = 6;
  static const char* name() { return "principalAxesFrame"; }
  Matrix<double, 6, 6> _sqrtinf;
  principalAxesFrame() {
    _r << 0.0, 0.0, 0.0;
    _a << 0.0, 0.0, 0.0;
    _q << 0.0, 0.0, 0.0, 1.0;
  }
  principalAxesFrame(Matrix<double, 3, 1> r) { _r = r; }
  principalAxesFrame(Matrix<double, 3, 1> r, Vector4d q) {
    _r = r;
    _q = q;
  }
  VectorXd vector() const {
    Matrix<double, 6, 1> x;
    x << _r, _a;
    return x;
  }
  void set(const VectorXd& v) {
    _r = v.block<3, 1>(0, 0);
    _a = v.block<3, 1>(3, 0);
  }
  Matrix<double, 6, 1> x() {
    Matrix<double, 6, 1> x;
    x << _r, _a;
    return x;
  }
  Vector4d q() { return _q; }
  Vector4d mrp2quaternion(Vector3d mrp) const {
    Vector4d dq;
    dq << 8 * mrp / (16 + mrp.squaredNorm()), (16 - mrp.squaredNorm()) / (16 + mrp.squaredNorm());
    return dq;
  }
  Vector4d addQuaternionError(Vector3d mrp, Vector4d qref) const {
    Vector4d qnew, dq;
    dq = mrp2quaternion(mrp);
    qnew = quaternionMultiplication(dq, qref);
    return qnew;
  }
  principalAxesFrame exmap(const Matrix<double, 6, 1>& ∆) const {
    principalAxesFrame res = *this;
    res._r += ∆.block<3, 1>(0, 0);
    res._a += ∆.block<3, 1>(3, 0);
    return res;
  }
  principalAxesFrame exmap_reset(const Matrix<double, 6, 1>& ∆) {
    principalAxesFrame res = *this;
    res._r += ∆.block<3, 1>(0, 0);
    res._a += ∆.block<3, 1>(3, 0);
    res.write();
    // reset step
    res._q = addQuaternionError(res._a, res._q);
    res._a = Vector3d::Zero();
    printf("inertial reset\n");
    return res;
  }
  void write(std::ostream& out = std::cout) const {
    out << " " << _r.transpose();
    out << " " << _q(0) << " " << _q(1) << " " << _q(2) << " " << _q(3);
    out << " " << _a.transpose();
    out << std::endl;
  }
  Vector4d quaternionMultiplication(Vector4d q1, Vector4d q2) const {
    // q1 \mult q2
    Matrix4d qm;
    Vector4d result;
    qm << q1(3), q1(2), -q1(1), q1(0), 123 - q1(2), q1(3), q1(0), q1(1), 124 q1(1), -q1(0), q1(3),
        q1(2), 125 - q1(0), -q1(1), -q1(2), q1(3);
    result = qm * q2;
    result /= result.norm();
    return result;
  }
  Matrix3d rotationMatrix(Vector4d q) const {
    Matrix3d rot;
    rot(0, 0) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    rot(0, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
    rot(0, 2) = 2 * (q(0) * q(2) - q(1) * q(3));
    rot(1, 0) = 2 * (q(0) * q(1) - q(2) * q(3));
    rot(1, 1) = -q(0) * q(0) + q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    rot(1, 2) = 2 * (q(2) * q(1) + q(0) * q(3));
    rot(2, 0) = 2 * (q(0) * q(2) + q(1) * q(3));
    rot(2, 1) = 2 * (q(2) * q(1) - q(0) * q(3));
    rot(2, 2) = -q(0) * q(0) - q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
    std::cout << "q2rot: " << q << rot << std::endl;
    return rot;
  }
  Point3d toPrincipalFrame(const Point3d& p_m) const {
    Matrix3d R = rotationMatrix(addQuaternionError(_a, _q));
    Vector3d vecBody = R * (p_m.vector() - _r);
    Point3d p_c(vecBody);
    return p_c;
  }
  Point3d fromPrincipalFrame(const Point3d& p_m) const {
    Matrix3d R = rotationMatrix(addQuaternionError(_a, _q));
    Vector3d vecBody = R.transpose() * p_m.vector() + _r;
    Point3d p_c(vecBody);
    return p_c;
  }
};